#!/usr/bin/env python3

import os, sys
import time
import rospy

import torch


def main():
    model = torch.jit.load('/home/mattia/pf-training/SerializedModels/coverage_model.pt')
    model.eval()
    print("Model loaded:")
    print(model)


if __name__ == "__main__":
    main()