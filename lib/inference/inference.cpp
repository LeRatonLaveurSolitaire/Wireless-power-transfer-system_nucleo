#include "inference.h"
#include "model_float32.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#define NUM_TF_OPS 2    // this is the number of ops in the model, used by the
                        // micro_mutable_op_resolver

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h"
#include "tensorflow/lite/schema/schema_generated.h"

#define RAM_SIZE 2000 // 2000 bytes  (start large and progressively reduce)
uint8_t tensor_arena[RAM_SIZE];

// Globally accessible interpreter
std::unique_ptr<tflite::MicroInterpreter> interpreter;

//#include  <Serial.h>

void tflite_setup(){
    
  // set up the error reporter
  static tflite::MicroErrorReporter micro_error_reporter;
  tflite::ErrorReporter *error_reporter = &micro_error_reporter;

  // set up the model
  const tflite::Model *model = tflite::GetModel(model_float32);
  // check to make sure the model is compatible
  if (model->version() != TFLITE_SCHEMA_VERSION) {
// #ifdef ARDUINO // serial out for Arduino instead of stdout
//     Serial.print("Model provided is schema version ");
//     Serial.print(model->version());
//     Serial.print(" not equal to supported version ");
//     Serial.println(TFLITE_SCHEMA_VERSION);
// #endif
    TF_LITE_REPORT_ERROR(error_reporter,
                         "Model provided is schema version %d not equal to "
                         "supported version %d.",
                         model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  // set up the optional micro mutable ops resolver, and add needed operations
  static tflite::MicroMutableOpResolver<NUM_TF_OPS> resolver;
  resolver.AddRelu();
  resolver.AddFullyConnected();

  // Declare the TF lite interpreter
  static tflite::MicroInterpreter static_interpreter(model, resolver,
                                                     tensor_arena, RAM_SIZE);
  interpreter = std::unique_ptr<tflite::MicroInterpreter>(&static_interpreter);

  // Allocate memory for the model's input buffers
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(error_reporter, "Tensor allocation failed");
  } else {
    // Serial.println("Tensor allocation success");
    // Serial.print("Used bytes: ");
    // Serial.println(interpreter->arena_used_bytes());
  }
}

void inference(float_t * input_data, float_t* output_data){
    
  static tflite::MicroErrorReporter micro_error_reporter;
  tflite::ErrorReporter *error_reporter = &micro_error_reporter;

  TfLiteTensor *input = interpreter->input(0);
  for (unsigned int i = 0; i < input->bytes; i++) {
    input->data.f[i] = input_data[i];
  }

  // Invoke the model

  TfLiteStatus invoke_status = interpreter->Invoke();

  if (invoke_status != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed");
  } else {
    // Serial.println("Invoke completed");
  }

  // Obtain a pointer to the model's output tensor
  TfLiteTensor *output = interpreter->output(0);
  float R = output->data.f[0];
  float M = output->data.f[1];

  R = pow(10, R * 0.15 + 0.5);
  M = pow(10, (0.1 * M)) * (0.1 * 33.727140406503484);
  output_data[0] = R;
  output_data[1] = M;
}