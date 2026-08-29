#!/usr/bin/env python3
"""Example 06: Vision-Language-Action (VLA) Natural Language Task Grounding."""

import numpy as np
from stacc.ai import VLATaskGrounder, VisuomotorPolicyRuntime


def main():
    print("=" * 60)
    print("STACC ROBOTICS: VLA Natural Language Grounding & Policy Chunking")
    print("=" * 60)

    # 1. Vision-Language-Action Task Grounder
    vla = VLATaskGrounder()
    prompt = "Pick up the bronze gear from the bin and place it into the gearbox housing"
    print(f"Input Prompt: '{prompt}'")

    plan = vla.decompose_instruction(prompt)
    print("\nDecomposed Action Plan:")
    for i, step in enumerate(plan, 1):
        print(f"  Step {i}: {step['action']} -> {step}")

    # 2. Visuomotor Diffusion Policy Runtime
    policy = VisuomotorPolicyRuntime(action_dim=7, chunk_horizon=16)
    dummy_camera_rgb = np.zeros((224, 224, 3), dtype=np.uint8)
    q_proprioception = np.array([0.1, -0.4, 0.8, 0.0, 0.5, 0.0])

    print("\nExecuting Action Chunking Steps:")
    for step_idx in range(4):
        action = policy.predict_action(dummy_camera_rgb, q_proprioception, gripper_state=1.0)
        print(f"  Step {step_idx + 1} Action Vector: {np.round(action, 3)}")


if __name__ == "__main__":
    main()
