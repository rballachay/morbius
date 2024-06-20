import sys
import torch
import numpy as np

sys.path.append('submodules/RTFNet')
from model import RTFNet
model = RTFNet(n_class=9)
model.load_state_dict(torch.load('/Users/RileyBallachay/Downloads/final.pth', map_location='cpu'))
model.eval()


torch_input = torch.randn(1, 4, 480, 640)

#output = model(torch_input)
onnx_program = torch.onnx.dynamo_export(model, torch_input)

onnx_program.save("RTFNet.onnx")