import onnx

# Load the original model
model = onnx.load("best.onnx")

# Optionally set the opset version (e.g., to 15)
model.opset_import[0].version = 15  # Adjust this version as necessary

# Save the model
onnx.save(model, "best_compatible.onnx")