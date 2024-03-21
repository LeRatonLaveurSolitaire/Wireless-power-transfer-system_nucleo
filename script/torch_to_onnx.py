import torch
from nn_model import NN_model


onnx_model_path = "model_no_Z1.onnx"

model_path = "script/model_noise.pt"


print("Loading model...")

model = NN_model(input_size=30, output_size=2)
checkpoint = torch.load(model_path)
model.load_state_dict(checkpoint)
print("Model sucessfully loaded !\n")

model.eval()

sample_input = torch.rand(1, 30)
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
