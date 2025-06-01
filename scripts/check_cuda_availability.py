import torch

def main():
    return not torch.cuda.is_available()