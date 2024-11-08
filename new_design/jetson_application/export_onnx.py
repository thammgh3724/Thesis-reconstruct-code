import torch

# Load the saved model (.pt file)
model = torch.load("best4.pt", map_location=torch.device("cuda" if torch.cuda.is_available() else "cpu"))
model.eval()  # Set the model to evaluation mode

# Create a dummy input tensor with a fixed shape as an example
dummy_input = torch.randn(1, 3, 640, 640)  # Adjust this to match the model's expected input shape

# Define dynamic axes for input and output
dynamic_axes = {
    "input": {0: "batch_size", 2: "height", 3: "width"},  # Make batch, height, width dynamic
    "output": {0: "batch_size"}  # Example of dynamic batch size for output (adjust as needed)
}

# Export the model to ONNX with dynamic shape support
torch.onnx.export(
    model, 
    dummy_input, 
    "model_dynamic.onnx",  # File name for the exported ONNX model
    export_params=True,  # Store learned parameters in the model
    opset_version=11,    # Specify ONNX version, adjust if needed
    do_constant_folding=True,  # Perform constant folding for optimization
    input_names=["input"],  # Name of the input tensor
    output_names=["output"],  # Name of the output tensor
    dynamic_axes=dynamic_axes  # Apply dynamic axes
)

print("ONNX model exported with dynamic shape support!")
