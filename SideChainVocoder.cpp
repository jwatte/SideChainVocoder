
#include "audioeffectx.h"
#include "aeffeditor.h"

#include <stdio.h>  //  for sprintf
#include <assert.h>
#include <windows.h>  //  for OutputDebugString
#include <math.h>

#undef min
#undef max

#include <algorithm>

#include "mkl_dfti.h"

#ifndef __vstgui__
#include "vstgui.sf/vstgui/vstgui.h"
#endif
#include "resource.h"

#undef min
#undef max

#if !defined(STRINGIZE)
#define STRINGIZE(x) STRINGIZE2(x)
#define STRINGIZE2(x) #x
#endif


//  TODO:
//
//  - 64 bit support
//  - higher sample rates -> more FFT samples
//    - 512 at > 50 kHz
//    - 1024 at > 100 kHz
//    - expand the table in the binning


//#pragma optimize("g", off)


#define FFT_SAMPLES 256


class SideChainVocoder;

using namespace VSTGUI;

class VocoEditor : public AEffGUIEditor, public CControlListener {
  public:
    VocoEditor(SideChainVocoder *scv);
    ~VocoEditor();
    bool getRect(ERect **oRect);
    bool open(void *hwnd);
    void close();
    void valueChanged(CDrawContext *ctx, CControl *ctl);

    ERect myRect_;
    CControl *modeSwitch_;
};

#define NUM_PROGRAMS 8
#define NUM_PARAMS 1

class SideChainVocoder : public AudioEffectX {
  public:
    SideChainVocoder(audioMasterCallback callback)
      : AudioEffectX(callback, NUM_PROGRAMS, NUM_PARAMS)
      , bypass_(false)
      , summing_(false)
    {
      numBlocks_ = 0;
      numSamples_ = 0;
      clearBuffers(true);
      setUniqueID('scVC');
      canProcessReplacing();
      setNumInputs(2);
      setNumOutputs(2);
      setInitialDelay(FFT_SAMPLES/2);
      setEditor(new VocoEditor(this));
      numParams_ = NUM_PARAMS;
      numPrograms_ = NUM_PROGRAMS;
      memset(programs_, 0, sizeof(programs_));
      for (int i = 0; i < NUM_PROGRAMS; ++i) {
        _snprintf(programs_[i].name, sizeof(programs_[i].name), "Program %c", 'A' + i);
        programs_[i].name[kVstMaxProgNameLen-1] = 0;
        programs_[i].mode = (i & 1) ? MixSelectRight : MixSelectLeft;
      }
    }

    bool bypass_;
    bool summing_;
    int numBlocks_;
    long long numSamples_;

    float input_buf[2][FFT_SAMPLES];
    float output_buf[2][2][FFT_SAMPLES];
    unsigned int buf_ptr_;
    DFTI_DESCRIPTOR_HANDLE forwardHandle;
    DFTI_DESCRIPTOR_HANDLE backwardHandle;

    void clearBuffers(bool andPointer) {
      memset(output_buf, 0, sizeof(output_buf));
      memset(input_buf, 0, sizeof(input_buf));
      if (andPointer) buf_ptr_ = 0;
    }

	  virtual void open();		  ///< Called when plug-in is initialized
	  virtual void close();	    ///< Called when plug-in will be released
	  virtual void suspend();	  ///< Called when plug-in is switched to off
	  virtual void resume();	  ///< Called when plug-in is switched to on
	  virtual void processReplacing(float** inputs, float** outputs, VstInt32 sampleFrames);

    //  Implement process() in terms of processReplacing() by running through small buffers
    //  in blocks into temp buffers, and adding into the actual destination.
	  virtual void DECLARE_VST_DEPRECATED(process)(float** inputs, float** outputs, VstInt32 sampleFrames) {
	    if (!summing_) {
	      ::OutputDebugStringA("Processing in summing mode.\n");
	    }
	    summing_ = true;
	    float op[2][64];
	    float *out[2] = { op[0], op[1] };
	    float *in[2] = { inputs[0], inputs[1] };
	    VstInt32 accum = 0;
	    while (sampleFrames > 0) {
	      VstInt32 todo = sampleFrames;
	      if (todo > 64) {
	        todo = 64;
	      }
	      processReplacing(in, out, todo);
	      for (VstInt32 i = 0; i < todo; ++i) {
	        outputs[0][accum+i] += out[0][i];
	        outputs[1][accum+i] += out[1][i];
	      }
	      in[0] += todo;
	      in[1] += todo;
	      accum += todo;
	      sampleFrames -= todo;
	    }
	  }

	  virtual bool getInputProperties(VstInt32 index, VstPinProperties* properties); ///< Return the \e properties of output \e index
	  virtual bool getOutputProperties(VstInt32 index, VstPinProperties* properties);///< Return the \e properties of input \e index
	  virtual bool setBypass(bool onOff);				///< For 'soft-bypass' (this could be automated (in Audio Thread) that way you could NOT call iochanged (if needed) in this function, do it in fxidle).

	  virtual bool getEffectName(char* name) { strcpy(name, "SideChainVocoder"); return true; }	///< Fill \e text with a string identifying the effect
	  virtual bool getVendorString(char* text) { strcpy(text, "Enchanted Age Productions"); return true; }	///< Fill \e text with a string identifying the vendor
	  virtual bool getProductString(char* text) { strcpy(text, "Enchanted Age Side Chain Vocoder 0.1"); return true; }///< Fill \e text with a string identifying the product name
	  virtual VstInt32 getVendorVersion() { return 100; }			///< Return vendor-specific version
	  virtual VstInt32 canDo(char* text);			///< Reports what the plug-in is able to do (#plugCanDos in audioeffectx.cpp)
	  virtual VstPlugCategory getPlugCategory() { return kPlugCategEffect; }

    enum MixSelector {
      MixSelectBlend,
      MixSelectLeft,
      MixSelectRight,
      MixSelectStereo,
    };
    struct Program {
      char name[kVstMaxProgNameLen];
      int  mode;
    };
    int numParams_;
    int numPrograms_;
    Program programs_[NUM_PROGRAMS];

    void loadProgram();
    void updateGui();
	  static char const *getMixStr(int value) {
	    switch (value) {
	      case 1: return "Left";
	      case 2: return "Right";
	      case 3: return "Stereo";
	      default: return "Blend";
	    }
	  }

	  virtual void setProgram (VstInt32 program) {
	    AudioEffectX::setProgram(program);
	    loadProgram();
	  }
	  virtual void setProgramName (char* name) {
	    _snprintf(programs_[curProgram].name, kVstMaxProgNameLen, "%s", name);
	    programs_[curProgram].name[kVstMaxProgNameLen-1] = 0;
	  }
	  virtual void getProgramName (char* name) {
	    _snprintf(name, kVstMaxProgNameLen, "%s", programs_[curProgram].name);
	    name[kVstMaxProgNameLen-1] = 0;
	  }
	  virtual void getParameterLabel (VstInt32 index, char* label)  {
	    _snprintf(label, kVstMaxParamStrLen, "%s", "");
	    label[kVstMaxParamStrLen-1] = 0;
	  }
	  virtual void getParameterDisplay (VstInt32 index, char* text) {
	    _snprintf(text, kVstMaxParamStrLen, "%s", getMixStr(programs_[curProgram].mode));
	    text[kVstMaxParamStrLen-1] = 0;
	  }
	  virtual void getParameterName (VstInt32 index, char* text)    {
	    _snprintf(text, kVstMaxParamStrLen, "%s", "Select");
	    text[kVstMaxParamStrLen-1] = 0;
	  }
	  virtual VstInt32 getChunk (void** data, bool isPreset = false) {
	    *data = &numParams_;
	    return (VstInt32)((char *)&programs_[NUM_PROGRAMS] - (char *)&numParams_);
	  }
	  virtual VstInt32 setChunk (void* data, VstInt32 byteSize, bool isPreset = false) {
	    size_t diff = (char *)programs_ - (char *)&numParams_;
	    if (byteSize < 0 || (size_t)byteSize < diff) return 0;
	    memcpy(&numParams_, data, diff);
	    if (numParams_ != NUM_PARAMS || numPrograms_ > NUM_PROGRAMS) {
	      numParams_ = NUM_PARAMS;
	      numPrograms_ = NUM_PROGRAMS;
	      return 0;
	    }
      numParams_ = NUM_PARAMS;
      numPrograms_ = NUM_PROGRAMS;
	    memcpy(programs_, (char *)data + diff, std::min(byteSize - diff, sizeof(programs_)));
	    return (VstInt32)std::min((size_t)(diff + sizeof(programs_)), (size_t)byteSize);
	  }

    void setParameter(int index, float value) {
      programs_[curProgram].mode = (int)floorf(value + 0.1f);
      updateGui();
    }
    float getParameter(int index) { return (float)programs_[curProgram].mode; }
};


AudioEffect *createEffectInstance(audioMasterCallback callback) {
  return new SideChainVocoder(callback);
}


void SideChainVocoder::open() {
  long status = DftiCreateDescriptor(&forwardHandle, DFTI_SINGLE, DFTI_REAL, 1, FFT_SAMPLES);
  DftiSetValue(forwardHandle, DFTI_PACKED_FORMAT, DFTI_PERM_FORMAT);
  status = DftiSetValue(forwardHandle, DFTI_REAL_STORAGE, DFTI_REAL_REAL);
  status = DftiSetValue(forwardHandle, DFTI_BACKWARD_SCALE, 2.0f / (FFT_SAMPLES*FFT_SAMPLES));
  status = DftiSetValue(forwardHandle, DFTI_PLACEMENT, DFTI_NOT_INPLACE);
  status = DftiCommitDescriptor(forwardHandle);

  status = DftiCreateDescriptor(&backwardHandle, DFTI_SINGLE, DFTI_REAL, 1, FFT_SAMPLES);
  DftiSetValue(backwardHandle, DFTI_PACKED_FORMAT, DFTI_PERM_FORMAT);
  status = DftiSetValue(backwardHandle, DFTI_REAL_STORAGE, DFTI_REAL_REAL);
  status = DftiSetValue(backwardHandle, DFTI_BACKWARD_SCALE, 1.0f / (FFT_SAMPLES*FFT_SAMPLES));
  status = DftiCommitDescriptor(backwardHandle);
}

void SideChainVocoder::close() {
  DftiFreeDescriptor(&forwardHandle);
  DftiFreeDescriptor(&backwardHandle);
}

void SideChainVocoder::suspend() {
  char str[128];
  sprintf(str, "blocks %d samples %I64d\n", numBlocks_, numSamples_);
  ::OutputDebugStringA(str);
  summing_ = false;
}

void SideChainVocoder::resume() {
  char status[256];
  sprintf(status, "SideChainVocoder::resume(FFT_SAMPLES=%d, sampleRate=%g)\n", FFT_SAMPLES, (float)sampleRate);
  ::OutputDebugStringA(status);
  clearBuffers(true);
}

bool SideChainVocoder::setBypass(bool onOff) {
  bypass_ = onOff;
  if (onOff) {
    clearBuffers(false);
  }
  if (buf_ptr_ >= (FFT_SAMPLES >> 1)) {
    ::OutputDebugStringA("It's impossible for buf_ptr_ to be this big!\n");
    ::DebugBreak();
  }
  return true;
}


#define LEFT 0
#define RIGHT 1
#define PING 0
#define PONG 1

void SideChainVocoder::processReplacing(float** inputs, float** outputs, VstInt32 sampleFrames) {
  if (buf_ptr_ >= (FFT_SAMPLES >> 1)) {
    ::OutputDebugStringA("It's impossible for buf_ptr_ to be this big!\n");
    ::DebugBreak();
  }
  if (bypass_) {
    for (int i = 0; i < 2; ++i) {
      memcpy(outputs[i], inputs[i], sampleFrames * 4);
    }
    buf_ptr_ += sampleFrames;
    buf_ptr_ &= ((FFT_SAMPLES >> 1) - 1);
  }
  else {
    numSamples_ += sampleFrames;
    numBlocks_ += 1;
    float const *inputs0 = inputs[0];
    float const *inputs1 = inputs[1];
    float *outputs0 = outputs[0];
    float *outputs1 = outputs[1];
    for (int i = 0; i < sampleFrames; ++i) {
      float o0 = 0, o1 = 0;
      float i0 = inputs0[i];
      float i1 = inputs1[i];

      input_buf[0][buf_ptr_+(FFT_SAMPLES>>1)] = i0;
      input_buf[1][buf_ptr_+(FFT_SAMPLES>>1)] = i1;
      float scl = buf_ptr_ / (float)(FFT_SAMPLES>>1);
      o0 = output_buf[PING][LEFT][buf_ptr_] * scl + output_buf[PONG][LEFT][buf_ptr_] * (1-scl);
      o1 = output_buf[PING][RIGHT][buf_ptr_] * scl + output_buf[PONG][RIGHT][buf_ptr_] * (1-scl);
      buf_ptr_++;
      if (buf_ptr_ == FFT_SAMPLES >> 1) {
        memcpy(output_buf[PONG][LEFT], output_buf[PING][LEFT]+(FFT_SAMPLES>>1), sizeof(output_buf[PONG][LEFT])/2);
        memcpy(output_buf[PONG][RIGHT], output_buf[PING][RIGHT]+(FFT_SAMPLES>>1), sizeof(output_buf[PONG][RIGHT])/2);

        long status = DftiComputeForward(forwardHandle, input_buf[LEFT], output_buf[PING][LEFT]);
        status = DftiComputeForward(forwardHandle, input_buf[RIGHT], output_buf[PING][RIGHT]);

        memcpy(input_buf[LEFT], input_buf[LEFT]+(FFT_SAMPLES>>1), sizeof(input_buf[LEFT])/2);
        memcpy(input_buf[RIGHT], input_buf[RIGHT]+(FFT_SAMPLES>>1), sizeof(input_buf[RIGHT])/2);

        //  munge the data
        float b0 = output_buf[PING][LEFT][0] * output_buf[PING][RIGHT][0];
        float b1 = output_buf[PING][LEFT][1] * output_buf[PING][RIGHT][1];
        output_buf[PING][LEFT][0] = b0;
        output_buf[PING][RIGHT][0] = b0;
        output_buf[PING][LEFT][1] = b1;
        output_buf[PING][RIGHT][1] = b1;
        //  todo: this would be totally easy to accelerate using SSE
        float tmp[2][FFT_SAMPLES/2];
        for (int i = 2; i < FFT_SAMPLES; i += 2) {
          float l0x = output_buf[PING][LEFT][i];
          float l0y = output_buf[PING][LEFT][i+1];
          float l1x = output_buf[PING][RIGHT][i];
          float l1y = output_buf[PING][RIGHT][i+1];
          float l0 = sqrtf(l0x*l0x+l0y*l0y);
          float l1 = sqrtf(l1x*l1x+l1y*l1y);
          tmp[LEFT][i>>1] = l0;
          tmp[RIGHT][i>>1] = l1;
        }
        //  This array defines the binning of different FFT samples into 
        //  bands for the vocoder. As it is, we have 13 bands on a 48 kHz 
        //  vocoder. Doubling the frequency would give us 15 bands, and 
        //  doubling again would give us 17 bands.
        static int const start[] = {
          1, 2, 3, 4, 6, 8, 12, 16, 24, 32, 48, 64, 96, 128,
        };
        for (int i = 0; start[i] < (FFT_SAMPLES>>1); ++i) {
          float sumL = 0, sumR = 0;
          for (int j = start[i], k = start[i+1]; j != k; ++j) {
            sumL += tmp[LEFT][j];
            sumR += tmp[RIGHT][j];
          }
          sumL *= 1.0f / (start[i+1] - start[i]);
          sumR *= 1.0f / (start[i+1] - start[i]);
          for (int j = start[i], k = start[i+1]; j != k; ++j) {
            output_buf[PING][LEFT][j*2] *= sumR;
            output_buf[PING][LEFT][j*2+1] *= sumR;
            output_buf[PING][RIGHT][j*2] *= sumL;
            output_buf[PING][RIGHT][j*2+1] *= sumL;
          }
        }

        //  put it back in the time domain
        status = DftiComputeBackward(backwardHandle, output_buf[PING][LEFT]);
        status = DftiComputeBackward(backwardHandle, output_buf[PING][RIGHT]);
        //  et voila, I start over from the beginning of the half-buffer
        buf_ptr_ = 0;
        switch (programs_[curProgram].mode) {
          case MixSelectBlend:
            for (int i = 0; i < FFT_SAMPLES; ++i) {
              float v = (output_buf[PING][LEFT][i] + output_buf[PING][RIGHT][i]) * 0.5f;
              output_buf[PING][LEFT][i] = v;
              output_buf[PING][RIGHT][i] = v;
            }
            break;
          case MixSelectLeft:
            memcpy(output_buf[PING][RIGHT], output_buf[PING][LEFT], sizeof(output_buf[PING][RIGHT]));
            break;
          case MixSelectRight:
            memcpy(output_buf[PING][LEFT], output_buf[PING][RIGHT], sizeof(output_buf[PING][LEFT]));
            break;
          case MixSelectStereo:
            //  do nothing
            break;
        }
      }
      outputs0[i] = o0;
      outputs1[i] = o1;
    }
  }
  if (buf_ptr_ >= (FFT_SAMPLES >> 1)) {
    ::OutputDebugStringA("It's impossible for buf_ptr_ to be this big!\n");
    ::DebugBreak();
  }
}

bool SideChainVocoder::getInputProperties(VstInt32 index, VstPinProperties* properties) {
  sprintf(properties->label, "Vocoder I%d", index+1);
  properties->flags = kVstPinIsActive | (kVstPinIsStereo * (1-index));
  properties->arrangementType = 0;
  sprintf(properties->shortLabel, "VocI%d", index+1);
  return true;
}

bool SideChainVocoder::getOutputProperties(VstInt32 index, VstPinProperties* properties) {
  sprintf(properties->label, "Vocoder O%d", index+1);
  properties->flags = kVstPinIsActive | (kVstPinIsStereo * (1-index));
  properties->arrangementType = 0;
  sprintf(properties->shortLabel, "VocO%d", index+1);
  return true;
}

VstInt32 SideChainVocoder::canDo(char* text) {
  if (!strcmp(text, "bypass"))
    return 1;
  return 0;
}

void SideChainVocoder::loadProgram() {
  updateGui();
}

void SideChainVocoder::updateGui() {
  if (((VocoEditor*)editor)->modeSwitch_ != 0)
    ((VocoEditor*)editor)->modeSwitch_->setValue((float)programs_[curProgram].mode / 3.0f);
}




VstIntPtr VSTCALLBACK master_callback(AEffect* effect, VstInt32 opcode, VstInt32 index, VstIntPtr value, void* ptr, float opt) {
  fprintf(stderr, "master_callback(%d, %d, %lld)\n", opcode, index, (long long)value);
  return 0;
}

int main(int argc, char const *argv[]) {
  if (argc != 3) {
    fprintf(stderr, "usage: voc input.wav output.wav\n");
    return 1;
  }
  FILE *input = fopen(argv[1], "rb");
  if (!input) {
    fprintf(stderr, "error: can't open %s\n", argv[1]);
    return 1;
  }
  FILE *output = fopen(argv[2], "wb");
  if (!output) {
    fprintf(stderr, "error: can't create %s\n", argv[2]);
    return 1;
  }
  SideChainVocoder *scv = new SideChainVocoder(master_callback);
  scv->open();
  scv->resume();

  int size = 0;
  int hdrSize = 0x1000;
  char hdr[0x1000];  //  octal!
  fread(hdr, 1, sizeof(hdr), input);
  for (int i = 0; i < 0x1000; ++i) {
    if (hdr[i] == 'd' && hdr[i+1] == 'a' && hdr[i+2] == 't' && hdr[i+3] == 'a') {
      size = *(int*)&hdr[i+4];
      hdrSize = i+8;
      break;
    }
  }
  fseek(input, hdrSize, 0);
  fwrite(hdr, 1, hdrSize, output);
  float leftin[512];
  float rightin[512];
  float leftout[512];
  float rightout[512];
  float *inputs[2] = { leftin, rightin };
  float *outputs[2] = { leftout, rightout };
  short inbuf[512][2];
  fprintf(stderr, "Processing %d bytes.\n", size);
  while (size > 0) {
    memset(inbuf, 0, sizeof(inbuf));
    fread(inbuf, 4, 512, input);
    for (int i = 0; i < 512; ++i) {
      leftin[i] = inbuf[i][0] / 32768.0f;
      rightin[i] = inbuf[i][1] / 32768.0f;
    }
    scv->processReplacing(inputs, outputs, 512);
    for (int i = 0; i < 512; ++i) {
      inbuf[i][0] = (int)(leftout[i] * 32767.0f);
      inbuf[i][1] = (int)(rightout[i] * 32767.0f);
    }
    fwrite(inbuf, 4, 512, output);
    if (size > 512)
      size -= 512;
    else
      size = 0;
  }

  fprintf(stderr, "Done processing.\n");
  scv->suspend();
  scv->close();
  fclose(input);
  fclose(output);
  delete scv;
  return 0;
}




VocoEditor::VocoEditor(SideChainVocoder *scv) : AEffGUIEditor(scv) {
  myRect_.left = 0;
  myRect_.top = 0;
  myRect_.right = 300;
  myRect_.bottom = 200;
  modeSwitch_ = 0;
}

VocoEditor::~VocoEditor() {
}

bool VocoEditor::getRect(ERect **rect) {
  *rect = &myRect_;
  return true;
}

bool VocoEditor::open(void *hwnd) {
  AEffEditor::open(hwnd);

  CRect fSize(0, 0, myRect_.right-myRect_.left, myRect_.bottom-myRect_.top);
  CFrame *tFrame = new CFrame(fSize, hwnd, this);

  CBitmap *bg = new CBitmap(IDB_BITMAP1);
  tFrame->setBackground(bg);
  bg->forget();

  bg = new CBitmap(IDB_BITMAP2);
  CHorizontalSwitch *hsw = new CHorizontalSwitch(CRect(100, 100, 200, 125), this, 0, 4, 25, 4, bg, CPoint(0, 0));
  tFrame->addView(hsw);
  modeSwitch_ = hsw;
  bg->forget();

  frame = tFrame;

  ((SideChainVocoder*)effect)->updateGui();

  return true;
}

void VocoEditor::close() {
  delete frame;
  frame = 0;
  modeSwitch_ = 0;
  AEffEditor::close();
}

void VocoEditor::valueChanged(CDrawContext *ctx, CControl *ctl) {
  effect->setParameterAutomated(0, floorf(ctl->getValue() * 3 + 0.1f));
  ctl->setDirty();
}

