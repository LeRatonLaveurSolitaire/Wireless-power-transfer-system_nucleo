import torch
from nn_model import NN_model


onnx_model_path = "model_32.onnx"

model_path = "script/model_32_inputs.pt"


print("Loading model...")

model = NN_model(input_size=64, output_size=2)
checkpoint = torch.load(model_path)
model.load_state_dict(checkpoint)
print("Model sucessfully loaded !\n")

model.eval()

sample_input = torch.rand(1, 64)
print(sample_input)
y = model(sample_input)

torch.onnx.export(
    model,
    sample_input,
    onnx_model_path,
    verbose=True,
    input_names=["input"],
    output_names=["output"],
    opset_version=12,
)
