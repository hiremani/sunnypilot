# Rendering and perception methods for a real-time 3D surroundings view (forward-camera car, RK3588S, WebGL/three.js)

Researcher notes, 2026-11-23. Scope: methods, inputs, compute cost, maturity, licences, open-source availability.

Reading rules for the report writer:
- Every item under "Cited Findings" was seen in a source this session. Most were read through search snippets or a page summariser, not the full primary page, so figures should be treated as "as reported by the source" and re-checked before they drive a purchase or licence decision.
- "Inferences" are my reasoning from the cited findings. They are not sourced facts.
- "Gaps" lists what I could not source. Several are important (no RK3588 numbers for most models, no Mali-G610 splat numbers).
- I stopped searching at about 22 tool calls, above the 15-call budget. Coverage of map tooling and hobby projects is thinner than coverage of perception models.

---

## 1. Camera-to-3D methods: which work with a single forward camera, and which need surround cameras?

### Takeaway
Monocular metric depth (Depth Anything V2/V3, Metric3D v2, UniDepth V2) and monocular semantic occupancy (MonoScene/VoxFormer family) work from one forward camera; the nuScenes-style occupancy, BEV and feed-forward Gaussian-splat driving methods (FlashOcc, DrivingForward, Omni-Scene, STORM) are designed and trained for surround rigs and public weights do not transfer to a single forward camera without retraining. For this car, per-frame depth is the only camera-to-3D family with public weights, permissive licences (small variants) and a measured embedded path.

### Cited Findings

Monocular depth (single camera: yes)
- Depth Anything V2 Small: 24.7M parameters, 94.3 MB float model, 518x518 input in Qualcomm's packaged build — [Qualcomm HF model card](https://huggingface.co/qualcomm/Depth-Anything-V2)
- Only the Small variant of Depth Anything V2 carries an Apache-2.0 licence (as stated on a community RK3588 conversion card) — [anguoyang RK3588 card](https://huggingface.co/anguoyang/depth-anything-v2-small-rknn-rk3588)
- Depth Anything 3 (arXiv Nov 2025) ships several checkpoints with mixed licences: DA3METRIC-LARGE (0.35B params, monocular metric depth) is listed Apache 2.0 on a Hugging Face card; the Nested-Giant-Large card is CC BY-NC 4.0 (non-commercial) — [DA3METRIC-LARGE card](https://huggingface.co/APaul1/DA3METRIC-LARGE); [DA3NESTED-GIANT-LARGE official card](https://huggingface.co/depth-anything/DA3NESTED-GIANT-LARGE); [DA3 paper](https://arxiv.org/pdf/2511.10647v1)
- A third-party table lists DA3 Base (0.12B) as Apache 2.0 and Large/Giant/Nested as CC BY-NC 4.0; one aggregator says all DA3 sizes are non-commercial, which contradicts the Apache cards — [third-party DA3 fork](https://github.com/KouseiChoki/da3); contradicted by [freeapihub aggregator](https://freeapihub.com/ai-models/depth-anything-3)
- DA3 metric output needs scaling by focal length in pixels to become metric depth (per a wrapper's README) — [Replicate DA3 metric wrapper](https://replicate.com/david20321/depth-anything-v3-metric-large/readme)
- UniDepth V2 code is released under CC BY-NC 4.0 (non-commercial), per a GitHub mirror of the repo — [unidepth mirror](https://github.com/leepms/unidepth)
- UniDepth V2 claims to be more efficient in time and parameters than UniDepth V1, with no exact figures in the abstract — [UniDepthV2 arXiv](https://arxiv.org/html/2502.20110v1)
- KITTI zero-shot AbsRel figures differ by paper and protocol: UniDepthV2 0.076 in one 2026 comparison table; another 2026 paper reports UniDepth V2 8.58% and Metric3D V2 3.50% (without GT intrinsics) / 5.25% (with) — [arXiv 2605.11578](https://arxiv.org/pdf/2605.11578); [FoundationGeo arXiv 2607.11588](https://arxiv.org/pdf/2607.11588)
- Metric3D v2 is a zero-shot metric depth and surface normal model; architecture and efficiency details are in its supplementary material — [Metric3Dv2 arXiv](https://arxiv.org/pdf/2404.15506)
- A 2026 paper targets shrinking foundation monocular depth to tiny models ("DepthART: Scaling Foundation Monocular Depth to Tiny Models") — [arXiv 2607.17099](https://arxiv.org/pdf/2607.17099) (title only seen; not read)

Monocular semantic occupancy (single camera: yes, research grade)
- MonoScene is described as the pioneering monocular method (2D-to-3D feature projection along line of sight, then a 3D UNet); VoxFormer-S is the usual comparison baseline on SemanticKITTI — [MonoOcc arXiv](https://arxiv.org/pdf/2403.08766)
- LWMOcc (SAE 2025) is a lightweight monocular occupancy framework on SemanticKITTI: 83.2% fewer parameters in the 2D feature extractor and 82.6% faster inference, aimed at embedded deployment; no absolute FPS found — [LWMOcc SAE paper](https://saemobilus.sae.org/papers/lwmocc-lightweight-monocular-3d-occupancy-prediction-method-2025-01-7312)
- L2COcc: single-frame monocular occupancy with LiDAR-model distillation, over 23% lower memory and inference time than the prior state of the art (URL-to-title mapping not confirmed) — [arXiv 2503.12369](https://arxiv.org/pdf/2503.12369v1)

Surround-camera occupancy / BEV (single camera: no, as published)
- FlashOcc (Apache-2.0, weights on Google Drive) is trained on Occ3D-nuScenes; M0 (ResNet-50, 256x704): 31.95 mIoU, 39.94M params, 154.1 GFLOPs — [FlashOCC repo](https://github.com/Yzichen/FlashOCC); [FlashOcc paper](https://arxiv.org/pdf/2311.12058)
- FlashOcc speed is a desktop-GPU figure: 197.6 FPS TensorRT FP16 on an RTX 3090; 96.2 FPS TRT FP32; 383.7 FPS FP16+INT8 PTQ at 29.60 mIoU — [FlashOCC repo](https://github.com/Yzichen/FlashOCC)
- Panoptic-FlashOcc: about 29-31.6 mIoU at 30-44 FPS on an A100 with PyTorch FP32 (a different backend, not comparable to the TensorRT figures) — [FlashOCC repo](https://github.com/Yzichen/FlashOCC); [Panoptic-FlashOcc arXiv](https://export.arxiv.org/pdf/2406.10527)
- A separate C++/CUDA TensorRT implementation (TRT_FlashOcc) exists — [FlashOCC repo](https://github.com/Yzichen/FlashOCC)

Feed-forward Gaussian splatting for driving (single camera: no, as published)
- DrivingForward (AAAI 2025): feed-forward 3DGS from "flexible surround-view input"; pose, depth and Gaussian networks trained jointly, only depth and Gaussian networks run at inference; MIT licence; pretrained weights public (Google Drive); trained on nuScenes; described as real-time reconstruction with no ms figure on the repo page — [DrivingForward repo](https://github.com/fangzhou2000/DrivingForward); [DrivingForward arXiv](https://arxiv.org/html/2409.12753v2)
- STORM (NVlabs GaussianSTORM): feed-forward 4D street-scene reconstruction from sparse multi-view sequences; needs precomputed depth, flow and ground inputs; trained on Waymo; "NVIDIA License"; no official checkpoint released ("We cannot share an official checkpoint at this moment"), only an unofficial one referenced in issues — [GaussianSTORM repo](https://github.com/NVlabs/GaussianSTORM)
- STORM assumes near-constant object velocity (noted by a 2026 follow-up paper) — [arXiv 2605.11594](https://arxiv.org/html/2605.11594)
- Omni-Scene: "Omni-Gaussian" representation combining pixel-based and volume-based Gaussians for ego-centric sparse-view reconstruction; reports beating pixelSplat and MVSplat on that task — [Omni-Scene arXiv](https://arxiv.org/html/2412.06273v2)

### Inferences
- Decision table (my synthesis; "embedded real-time" means plausibly 5-20 Hz on the RK3588 NPU):

| Family | Works with 1 forward camera | Public weights | Licence (as found) | Embedded real-time on RK3588 | Status |
|---|---|---|---|---|---|
| Depth Anything V2 Small | Yes | Yes | Apache-2.0 | Not at 518x518 FP16 (about 1.4 fps measured); needs INT8 and lower resolution | Practical with work |
| DA3 Base / Metric-Large | Yes | Yes | Apache-2.0 (cards); verify | Large (0.35B) is too heavy; Base unmeasured | Verify licence, then test |
| UniDepth V2 | Yes | Yes | CC BY-NC 4.0 | Unmeasured | Non-commercial |
| Metric3D v2 | Yes | Yes (not verified this session) | Not found | Unmeasured | Unknown licence |
| Monocular occupancy (MonoScene, VoxFormer, LWMOcc) | Yes | Varies | Not checked | No evidence | Research only |
| FlashOcc / BEVDet-Occ | No (6-camera nuScenes training) | Yes | Apache-2.0 | No evidence; desktop TensorRT only | Research for this rig |
| BEVFormer / BEVFusion / TPVFormer / SurroundOcc / OccNet | No (surround; BEVFusion also LiDAR) | Not checked | Not checked | No | Desktop GPU research |
| DrivingForward | No (surround) | Yes | MIT | No evidence | Desktop GPU research |
| STORM | No (multi-view + precomputed depth/flow) | No official | NVIDIA License | No | Desktop GPU research |
| Omni-Scene, pixelSplat/MVSplat derivatives | No (multi-view) | Not checked | Not checked | No | Desktop GPU research |

- A nuScenes-trained surround model fed one forward camera is out of its training distribution; a single camera also cannot observe the side or rear volume that those models are built to fill. Using these methods on this car means retraining on forward-only data, which no source here provides.
- The wide + narrow comma road cameras share a forward view with a small baseline, so they do not count as "surround" for any of these methods. They could in principle supply a stereo or scale cue for depth, but I found no source that evaluates this.
- The front radar and the car's CAN object list give metric range to lead vehicles. They are a cheap way to fix the scale of a relative-depth network, which could allow a smaller non-metric depth model. This is an inference; no source tested it.

### Gaps
- BEVFormer, BEVFusion, TPVFormer, SurroundOcc, OccNet: I did not retrieve sources this session. I believe all are surround-camera (BEVFusion camera+LiDAR) nuScenes methods with desktop-GPU inference, but that is from prior knowledge, not verified here. Licences and weights not checked.
- pixelSplat and MVSplat themselves were only seen as baselines in the Omni-Scene abstract. Licences, weights and speed not checked.
- No absolute FPS or hardware figures found for MonoScene, VoxFormer, MonoOcc, LWMOcc or L2COcc.
- Metric3D v2 licence, parameter count and inference speed not found.
- DrivingForward and Omni-Scene inference time and GPU memory not found on the pages read.
- No source evaluated any of these with exactly "one wide + one narrow forward camera".
- 4D/dynamic reconstruction beyond STORM (for example per-scene methods such as Street Gaussians or OmniRe) was not researched; these are offline per-scene optimisation methods as far as I know, unverified here.

---

## 2. Accuracy, model size, and inference speed on embedded hardware (RK3588 NPU, Snapdragon, Jetson Orin), licences, public weights

### Takeaway
The only RK3588 measurement found for a modern depth model is poor: Depth Anything V2 Small at 518x518 unquantised takes about 696 ms per frame on the NPU (about 1.4 fps), from one self-published card. Qualcomm NPUs run the same model in roughly 19-75 ms float and 5-34 ms quantised depending on chip generation, and a Jetson AGX Orin reaches 44 FPS with ViT-S, so the RK3588 will need INT8 quantisation, a lower input resolution, or a smaller CNN depth network to reach 5-20 Hz.

### Cited Findings

RK3588 NPU
- Depth Anything V2 Small on RK3588: about 696 ms per image at 518x518, floating-point (no quantisation), 3 NPU cores, median of three sessions; model load about 250 ms; converted with rknn-toolkit2 2.3.2 from the official fp32 ONNX with static input; output correlates almost perfectly with the CPU ONNX reference (median relative error about 1.5%) — [anguoyang RK3588 card](https://huggingface.co/anguoyang/depth-anything-v2-small-rknn-rk3588)
- Caveats on that measurement: single self-published result, board was shared with a load of about 1.8-2.0 during the runs, and the card says no other RK3588 build of V2 existed on Hugging Face as of 2026-11-21 — [anguoyang RK3588 card](https://huggingface.co/anguoyang/depth-anything-v2-small-rknn-rk3588)
- For scale, a ResNet18 INT8 classifier runs in about 4.09 ms on the RK3588 NPU — [tinycomputers RK3588 NPU benchmarks](https://tinycomputers.io/posts/rockchip-rk3588-npu-benchmarks.html)
- Depth-Anything-V2-small is listed as supported on an RK3588 host with an RK1820/RK1828 co-processor over PCIe (no fps for that model in the listing) — [DFRobot wiki](https://wiki.dfrobot.com/dfr1263/docs/24735)
- GPU and NPU clocks on RK3588 are reported as not being set to nominal maximum in some kernel configurations; switching the governor to performance raised a glmark2-wayland score from about 28xx to 3093 — [Radxa forum](https://forum.radxa.com/t/16-more-gpu-performance-for-panthor/25260)
- Community developers ask for real on-device RKNN numbers rather than the 6 TOPS spec, which signals that published RK3588 benchmarks for newer vision models are scarce — [Ultralytics community thread](https://community.ultralytics.com/t/yolo26n-rknn-inference-speed-on-rk3588-orange-pi-5-plus-segmentation-alternatives/2131)

Qualcomm Snapdragon (Depth Anything V2 Small, 518x518, ONNX runtime on NPU; float ms / w8a16 quantised ms)
- Snapdragon 8 Gen 1: 74.6 / 33.5. Snapdragon 8 Gen 3: 34.2 / 19.3. Snapdragon 8 Elite: 26.0 / 14.7. QCS8550 (proxy): 44.5 / 27.1. QCS6490: quantised only, 85.0. Dragonwing Q-6690: quantised only, 207.9 — [Qualcomm HF model card](https://huggingface.co/qualcomm/Depth-Anything-V2)
- The table has no Snapdragon 845 and no Snapdragon 8 Gen 2 entry. A CPU-only automotive part (SA7255P, TFLite float) takes 1232 ms — [Qualcomm HF model card](https://huggingface.co/qualcomm/Depth-Anything-V2)
- Qualcomm's card metadata lists the licence as MIT and points to the original implementation's licence — [Qualcomm HF model card](https://huggingface.co/qualcomm/Depth-Anything-V2); differs from the Apache-2.0 statement on the [RK3588 card](https://huggingface.co/anguoyang/depth-anything-v2-small-rknn-rk3588)
- A hobby project reports Depth Anything V2 at over 11 fps NPU-only on a QCS9075 board — [Hackster project](https://www.hackster.io/gigwegbe/depth-estimation-on-qualcomm-dragonwing-boards-4526e0)
- Qualcomm AI Hub also packages Depth Anything V3 for mobile and automotive targets — [Qualcomm AI Hub DA3](https://aihub.qualcomm.com/automotive/models/depth_anything_v3)

Jetson
- Depth Anything on Jetson AGX Orin 64GB: ViT-S gives the best accuracy/real-time balance at 44 FPS, ViT-B degrades (KITTI evaluation; TensorRT use not confirmed in the snippet) — [UFRGS paper](https://seer.ufrgs.br/index.php/rita/article/download/150996/99006/700202)
- Real-time TensorRT ROS 2 wrappers for Depth Anything 3 on Jetson exist (community release) — [Open Robotics discourse](https://discourse.openrobotics.org/t/release-gerdsenais-depth-anything-3-ros2-wrapper-with-real-time-tensorrt-for-jetson/52364); [Seeed AGX Orin DA3 guide](https://wiki.seeedstudio.com/deploy_depth_anything_v3_jetson_agx_orin/)
- NVIDIA ships a TAO "nvdepthanythingv2" model with TensorRT deployment docs — [NGC catalog](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/tao/models/nvdepthanythingv2)

Occupancy (desktop only)
- FlashOcc M0: 31.95 mIoU Occ3D-nuScenes, 39.94M params, 154.1 GFLOPs, 197.6 FPS on RTX 3090 TensorRT FP16; INT8 PTQ costs about 2.4 mIoU (31.95 to 29.59) — [FlashOCC repo](https://github.com/Yzichen/FlashOCC)
- FlashOcc M3 (Swin-B, 512x1408, 2 frames): 43.52 mIoU, 144.99M params, 1490.77 GFLOPs — [FlashOCC repo](https://github.com/Yzichen/FlashOCC)

### Inferences
- 696 ms at 518x518 floating-point is roughly 20x slower than the Snapdragon 8 Gen 3 float figure (34 ms). The RK3588 NPU is strong on INT8 CNNs (ResNet18 in 4 ms) and weak on unquantised ViT workloads. A ViT-S depth model at 518x518 is therefore the wrong shape for this NPU; the options are (a) INT8 quantisation, (b) input around 256-384 px on the long side, (c) a CNN-backbone depth model, or (d) run depth at 1-2 Hz and reproject between frames using ego-motion.
- Cost scales roughly with pixel count for ViT models, so a 266x266 input is about 4x cheaper than 518x518 before any quantisation gain. Even so, reaching 10 Hz from 1.4 Hz needs about 7x, which probably requires both lower resolution and INT8. This is arithmetic from the cited 696 ms, not a measurement.
- The comma device already runs its own driving model on the NPU/GPU. Any added depth model competes for the same NPU and memory. The user's own project notes record that excess load has killed modeld mid-drive, so a visualisation network must be budgeted as strictly lower priority than driving.
- FlashOcc at 154 GFLOPs with six camera inputs is far outside a 6 TOPS NPU budget that is shared with a driving model, independent of the surround-camera problem.
- Licence summary for a commercial or redistributable product: Depth Anything V2 Small (Apache-2.0), FlashOcc (Apache-2.0), DrivingForward (MIT) are permissive; UniDepth V2 and the larger DA3 checkpoints are non-commercial; STORM is under an NVIDIA licence whose terms I did not read. Note that nuScenes-trained weights inherit dataset terms that I did not check.

### Gaps
- No RK3588 INT8 or reduced-resolution timing for Depth Anything V2 Small. This is the single most important missing number; the team should measure it on the board.
- No RK3588 figure for any occupancy, BEV, or Gaussian-splat network.
- No Snapdragon 845 figure for any of these models (relevant to comma 3/3X-class hardware).
- No Jetson Orin Nano figure for Depth Anything V2/V3; only AGX Orin (44 FPS ViT-S) and unnumbered Orin NX/Nano posts.
- Device names in the Qualcomm table (for example "Snapdragon 8 Elite Gen 6") are as returned by a page summariser; re-read the card before quoting chip names.
- Accuracy on real driving scenes at reduced resolution or after INT8 quantisation: not found.
- Metric3D v2 and UniDepth V2 embedded timings: not found.

---

## 3. Rendering options in WebGL/WebGPU on a Mali GPU

### Takeaway
No source measures Gaussian-splat rendering on a Mali-G610 or on RK3588 in a browser. Published mobile figures are for unnamed phones (about 38 fps WebGL2 and 78 fps WebGPU at 1M splats in PlayCanvas SuperSplat), three.js splat viewers report CPU-side sorting and weak mobile performance, and the RK3588's open GPU driver only reached OpenGL ES 3.1 conformance in mid-2024, so mesh and instanced-object rendering is the low-risk choice and splats are an unproven option on this board.

### Cited Findings

Gaussian splat renderers
- PlayCanvas SuperSplat on mobile (device not named): WebGL2 38.1 fps at 1M splats falling to 20.4 fps at 4M; WebGPU 77.6 fps at 1M falling to 42.4 fps at 4M, roughly 2x at every splat count — [PlayCanvas blog](https://blog.playcanvas.com/new-in-supersplat-webgpu-and-streaming-bring-huge-performance-wins)
- WebSplatter (WebGPU) reports 1.2x to 4.5x speedups over state-of-the-art web viewers across devices — [WebSplatter arXiv](https://arxiv.org/html/2602.03207v1)
- three.js GaussianSplats3D community notes list sub-optimal mobile performance and a CPU-based splat sort as known issues — [three.js forum thread](https://discourse.threejs.org/t/3d-gaussian-splatting-in-three-js/57858)
- Spark (sparkjsdev/spark) is a three.js Gaussian-splat renderer targeting WebGL2, advertised for desktop, mobile and WebXR and "optimized for low-powered mobile devices"; each frame it builds a global splat list, sorts back to front, then renders; it supports correct sorting across multiple splat objects and animated/dynamic splats; README advises `antialias: false` — [Spark repo](https://github.com/sparkjsdev/spark); [Spark docs](https://sparkjs.dev/docs/overview/)
- Spark 2.0 adds level of detail (fewer splats for distant content) and notes that mobile browsers cap GPU memory — [World Labs Spark 2.0 blog](https://worldlabs.ai/blog/spark-2.0)
- A W3C WebGPU splat sample notes it is hard to reach the desired performance on mobile without reducing splat count — [W3C splat demo](https://www.w3.org/2026/splat/)
- Same scene: 30-40 fps on iPhone 15 Pro versus 60-80 fps on a Samsung S22 in Babylon.js, showing large GPU/driver variance — [Babylon.js forum](https://forum.babylonjs.com/t/gaussian-splat-low-performance-on-ios-devices-vs-android/62455)
- Mobile-GPU splatting research identifies 2D texture-cache behaviour as a key optimisation target on mobile GPUs — [arXiv 2511.16298](https://arxiv.org/html/2511.16298v1)

RK3588 / Mali-G610 graphics stack
- The open Panthor kernel driver plus Mesa Panfrost reached OpenGL ES 3.1 conformance on Mali-G610 (tested on a Rock 5B) in July 2024 — [Collabora](https://www.collabora.com/news-and-blog/news-and-events/taming-the-panthor-opengl-es-31-conformance-achived-mali-g610.html); [CNX Software](https://www.cnx-software.com/2024/07/18/panthor-open-source-driver-achieves-opengl-es-3-1-conformance-with-arm-mali-g610-gpu-rk3588-soc/)
- glmark2-wayland on RK3588 with Panthor: about 2800-3100 depending on governor and clock source — [Radxa forum](https://forum.radxa.com/t/16-more-gpu-performance-for-panthor/25260)
- One Armbian user reported Chromium hanging the GUI when Vulkan was force-enabled on RK3588 (anecdotal, one setup) — [Armbian forum](https://forum.armbian.com/topic/47019-vulkan-driver/)

Other rendering references
- Uber's AVS (Autonomous Visualization System) is a web-based open standard and toolkit for autonomous-vehicle data visualisation — [Uber AVS blog](https://www.uber.com/us/en/blog/avs-autonomous-vehicle-visualization/)
- Viser is a web-based 3D visualisation library driven from Python — [Viser arXiv](https://arxiv.org/html/2507.22885v1)

### Inferences
- Options ranked by risk on Mali-G610 in a browser (my assessment):
  1. Object-level digital twin: instanced glTF vehicle/pedestrian/cyclist models, lane and road-edge ribbons from the driving model, extruded map buildings. Tens of thousands of triangles, no per-frame sorting, plain WebGL2. Lowest risk; this is also how automaker cluster visualisations look.
  2. Depth reprojection: one textured grid mesh or point cloud per camera frame (for example 160x120 = 19k vertices) displaced by the depth map in a vertex shader. Cheap to draw, but shows stretched "rubber sheet" artefacts at depth edges and holes when the virtual camera leaves the real camera's viewpoint.
  3. Gaussian splats: needs a per-frame sort of all splats. With CPU sorting on the RK3588's cores and an unnamed-phone baseline of about 38 fps at 1M splats in WebGL2, a budget in the low hundreds of thousands of splats is the most this board should be expected to hold at 30 fps, and that is a guess that must be measured. Splats also need a network to produce them; no such network runs on this hardware (section 1).
- WebGPU's roughly 2x advantage for splats is only useful if the browser on the device exposes WebGPU on the Mali driver in use. The one browser/Vulkan report found is negative, so WebGL2 should be the assumed baseline.
- A hybrid is feasible: object-level twin for everything dynamic, plus a pre-built static splat or mesh of a frequently driven route captured offline. I found no project doing this for a car display.
- The display rate and the perception rate are decoupled in all three options: the renderer can interpolate tracked object poses at 30 fps while perception updates at 5-20 Hz.

### Gaps
- No Gaussian-splat fps measurement on Mali-G610, on RK3588, or on any named Mali GPU.
- No three.js or WebGL2 scene benchmark in Chromium on RK3588 (only glmark2).
- Whether WebGPU is available and stable in the device's browser on Mali-G610: not found.
- mkkellogg/GaussianSplats3D and antimatter15/splat repos were not read directly (maintenance status, licence, sort implementation details unverified). Spark's actual splat-budget recommendations were not retrieved.
- Licences of Spark, GaussianSplats3D and PlayCanvas engine not confirmed this session.
- Which GPU driver (vendor Mali blob versus Panthor) and which browser/webview the target device actually uses: unknown; it changes every number above.

---

## 4. Map-based world building: OSM, Overture, Cesium/Google 3D tiles

### Takeaway
OSM-derived data (OSM itself, Overture buildings, Cesium OSM Buildings, Streets GL tiles, OSM2World output) is ODbL and can be stored offline with attribution, while Google Photorealistic 3D Tiles prohibit pre-fetching, caching and offline use, which rules them out for an in-car offline view. OSM2World can export glTF and 3D Tiles for use in three.js; Streets GL is a ready MIT-listed WebGL renderer but depends on its own tile server and has not been updated since September 2024.

### Cited Findings
- Google Map Tiles API policy: you must not pre-fetch, index, store, or cache content except under the limited conditions in the agreement; unauthorised caching, data extraction and offline use are prohibited; Google Maps attribution is required; different terms apply for EEA billing addresses — [Google Map Tiles API policies](https://developers.google.com/maps/documentation/tile/policies?authuser=1); [Photorealistic 3D Tiles docs](https://developers.google.com/maps/documentation/tile/3d-tiles?hl=en)
- Cesium OSM Buildings may include OpenStreetMap data under ODbL — [Cesium third-party terms](https://cesium.com/legal/third-party-terms/)
- Overture Maps buildings theme is ODbL because it contains OSM data; other sources in that theme must be ODbL-compatible (for example CC BY 4.0); Overture uses CDLA Permissive v2 where it can (for example Places) — [Overture buildings guide](https://docs.overturemaps.org/guides/buildings)
- Overture distributes GeoParquet on AWS S3 and Azure Blob Storage — [AWS open data registry](https://registry.opendata.aws/overture/)
- Overture's January 2024 release had 2.35B conflated building footprints — [Overture January 2024 release notes](https://overturemaps.org/overture-january-2024-release-notes/)
- One mirror describes the full Overture download (all themes, unspecified release) as "a little over 200 GB" — [GitHub mirror README](https://github.com/mogita/data)
- OSM2World is an open-source renderer that builds 3D worlds from OSM data; licence listed as LGPL-3.0; exports include glTF, OBJ and 3D Tiles; a WebGL client exists; its public demo covers Germany, Austria and Switzerland — [OSM wiki: OSM2World](https://wiki.openstreetmap.org/wiki/OSM2World); [OSM wiki: 3D](https://wiki.openstreetmap.org/wiki/3D); [OSM2World WebGL client](https://www.openstreetmap.org/wiki/OSM2World/WebGL_client)
- OSM2World is still active, with a FOSDEM 2026 talk — [FOSDEM 2026](https://fosdem.org/2026/schedule/event/osm2world_3d_rendering_openstreetmap_data)
- Streets GL is a web 3D map built on OSM as an open-source alternative to F4 Map and OSMBuildings; tiles come from a small Node.js server at tiles.streets.gl, generated with a modified Planetiler; one aggregator lists it as MIT; code last updated September 2024; self-described as early development — [streets-gl repo](https://github.com/strandedkitty/streets-gl); [OSM wiki: Streets GL](https://wiki.openstreetmap.org/wiki/Streets_GL); [ecosyste.ms listing](https://awesome.ecosyste.ms/projects/github.com%2Fstrandedkitty%2Fstreets-gl)

### Inferences
- For an offline in-car view the practical pipeline is: extract a regional OSM (or Overture buildings) cut on a desktop, convert to compact tiles (extruded footprints with height/levels, road polygons from way geometry plus `lanes`/`width` tags), ship tiles to the device, and extrude in three.js at load time. A region the size of a metro area is megabytes to low hundreds of megabytes as vector data, not the 200 GB planet. That size estimate is mine; no source gave regional figures.
- ODbL obligations that matter here: attribution on the display or in an about screen, and share-alike on any derived database that is distributed. Rendered imagery ("produced work") is treated more lightly than a redistributed database. I did not retrieve the ODbL text this session, so treat this as a pointer for a licence check.
- Map geometry gives what a forward camera cannot: buildings beside and behind the car, the road network beyond line of sight, junction layout, and stop signs/traffic signals where tagged. It gives nothing about dynamic objects or the true lane paint; lane count is only present where `lanes` is tagged.
- The user's existing project already consumes OSM (speed limits, stop signs), so tile extraction can share that data path.
- Map alignment is the main quality risk: GPS error of a few metres will visibly shift buildings and lanes relative to camera-detected objects. Snapping the map to the driving model's lane lines or to the road centreline is needed. No source read addressed this.

### Gaps
- OSMBuildings licence and offline terms, Mapbox GL / MapLibre GL fill-extrusion and 3D terrain, and MapLibre's licence (BSD, I believe) were not retrieved this session.
- Cesium ion pricing and offline terms, and whether Cesium OSM Buildings can be exported for offline use: not found.
- Overture buildings-only download size and per-region sizes: not found.
- Whether Streets GL can run fully offline with self-hosted tiles, and how it renders lanes: not found.
- OSM2World output size and triangle counts per square kilometre, and whether its glTF is light enough for Mali-G610: not found.
- OSM lane-level coverage (share of roads with `lanes`, `turn:lanes`, `width` tags) in the user's region: not found.
- ODbL text itself not read this session.

---

## 5. What realism is achievable with only a forward-facing camera?

### Takeaway
Tesla's visualisation is fed by an occupancy network that fuses eight surround cameras into one volumetric grid with per-voxel velocity; a forward-only rig observes only the forward cone, so everything beside and behind the car must come from maps, from tracking objects after they leave view, or be left out. A stylised object-level scene that is accurate ahead and map-plausible elsewhere is achievable; a photorealistic 360-degree view is not, on this sensor set.

### Cited Findings
- Tesla's occupancy network takes all eight cameras and builds a unified 3D volumetric representation around the car, predicting per-cell occupancy and, when occupied, a velocity vector (occupancy volume and occupancy flow) — [42how explainer](https://en.42how.com/2022/10/26/tesla-fsd-occupancy-network-explained/); [third-party Tesla perception notes](https://kvynlim.github.io/industry-research/80-industry-intel/companies/tesla/perception)
- Tesla's on-screen visualisation was originally a simplified display; older visualisations had trouble with objects spanning two camera views, which the occupancy approach handles better (forum commentary) — [Tesla Motors Club thread](https://teslamotorsclub.com/tmc/threads/tesla-replacing-ultrasonic-sensors-with-tesla-vision.281866/page-18); [Not a Tesla App](https://www.notateslaapp.com/software-updates/upcoming-features/id/1008/un)
- openpilot's radard matches the vision lead to a radar track when tracks exist and lead probability exceeds 0.5, so lead position/velocity is already a fused camera+radar estimate — [openpilot radard.py](https://github.com/commaai/openpilot/blob/master/selfdrive/controls/radard.py)
- openpilot logs modelV2, radarState and liveTracks at their own timestamps and rates, so a modelV2 frame may have no matching liveTracks sample — [openpilot PR 32850](https://github.com/commaai/openpilot/pull/32850)
- Feed-forward driving reconstruction methods that could synthesise unseen views assume surround input (DrivingForward) or multi-view sequences with precomputed depth and flow (STORM) — [DrivingForward repo](https://github.com/fangzhou2000/DrivingForward); [GaussianSTORM repo](https://github.com/NVlabs/GaussianSTORM)

### Inferences
- Breakdown of the scene by data source for this car (my analysis):

| Scene element | Source | Confidence |
|---|---|---|
| Ego lane lines, road edges, planned path ahead | Driving model output (camera) | Measured, forward only |
| Lead vehicle and near-lane vehicles ahead: range, speed | Radar + CAN ADAS object list + driving model leads | Measured |
| Vehicle class, size, heading, colour ahead | Needs an added detector on the camera image; otherwise a generic car model | Measured if detector added, else hallucinated |
| Pedestrians, cyclists, cones, signs, traffic-light state ahead | Needs an added detector; not in a radar/ADAS list in usable form | Not available without extra model |
| Vehicles alongside and behind | Not observed. Dead-reckon objects that were seen ahead and then passed (constant-velocity track with a decaying confidence), or omit | Memory, degrades in seconds |
| Vehicles approaching from behind, blind-spot traffic | Not observable at all unless the car's CAN exposes blind-spot/rear radar flags | Unavailable or flag only |
| Buildings, road network, junction geometry, static furniture | OSM/Overture map | Map prior; not live |
| Road surface behind the car | Accumulate what the camera saw as the car drove over it, or map | Memory |
| Sky, lighting, weather | Procedural from time/GPS | Hallucinated |
| Trees, parked cars not tagged in maps, construction | Forward depth/occupancy only while in view | Partial |

- The honest design is to show provenance: full-opacity for live-measured objects, faded or ghosted for dead-reckoned ones, and a neutral map style for map-only geometry. A view that draws remembered cars as solid will show cars that are no longer there.
- A chase-camera viewpoint above and behind the ego car is the worst case for a forward camera, because the visible area includes space behind the car that is never observed. A lower, forward-biased camera hides that area and makes the forward-only limitation less visible.
- Photorealism in the automaker sense (true textures of the actual surrounding cars and buildings) requires re-projecting camera pixels. With one forward camera this is only possible near the camera's own viewpoint; from a raised virtual camera the depth-reprojected image shows holes behind every object. The realistic target is a clean stylised scene, comparable to earlier automaker visualisations, not a photorealistic one.
- The driver camera does not help: it faces the cabin.

### Gaps
- No primary Tesla source (AI Day material, patent, release notes) was read; the Tesla description above is from third-party explainers and forum posts. I found no confirmation of how Tesla renders the scene (engine, assets), nor a source describing a feature named "Reality View".
- What the Fisker Ocean's CAN ADAS object list actually contains (number of objects, classes, side/rear radar or blind-spot signals) was not researched here; it decides how much of the side/rear can be shown.
- No study found quantifying how long a dead-reckoned out-of-view vehicle track stays useful.
- No source on monocular 3D object detection (class, size, heading) speed on RK3588; a detector would be needed for anything beyond generic car boxes.

---

## 6. Known open-source or hobby projects and their limits

### Takeaway
I found no open project that gives openpilot/comma a Tesla-style 3D surroundings view. What exists is developer tooling (openpilot's Rerun radar-point viewer and ui.py), 2D on-road themes in forks, a Honda dash-rendering patch, and unrelated three.js Tesla-styled demos, so this would be new work rather than an integration.

### Cited Findings
- openpilot has a Rerun-based tool (`tools/rerun/`) and an issue/PR to replace `ui.py`'s radar-point visualisation with a custom Rerun blueprint; the PR splits radar-point display into its own helper module — [openpilot issue 32837](https://github.com/commaai/openpilot/issues/32837); [openpilot PR 32850](https://github.com/commaai/openpilot/pull/32850)
- That issue is from mid-2024 (about 880 days old at time of search), so the current state of the Rerun integration is unverified — [openpilot issue 32837](https://github.com/commaai/openpilot/issues/32837)
- Community documentation describes the openpilot tool set as: the openpilot UI, watch3 for multi-camera view, PlotJuggler for plots, Rerun for radar points — [readmex openpilot summary](https://readmex.com/en-US/commaai/openpilot/page-10.43895db2b-9020-4073-8b68-4e8face577d2)
- A community opendbc PR renders openpilot lane lines and the lead car on the factory dash for Honda Bosch radarless cars (sends data to the OEM cluster, not a 3D scene) — [mvl-boston opendbc PR 588](https://github.com/mvl-boston/opendbc/pull/588)
- FrogPilot (openpilot fork) includes a "Tesla" theme among its custom themes; the result does not say it is a 3D scene (page seen was a mirror of the fork) — [FrogPilot mirror](https://github.com/mike86437/openpilot)
- mvimont/dash is a real-time motion planner and autonomous-vehicle simulator in the browser built on WebGL and three.js; it is a simulator, not a live-sensor UI — [mvimont/dash](https://github.com/mvimont/dash)
- A three.js forum project, "Tesla-inspired 3D DIY", is a Tesla-styled interactive 3D vehicle showcase and telemetry dashboard, not tied to openpilot or live perception — [three.js forum](https://discourse.threejs.org/t/tesla-inspired-3d-diy/92701)
- A Home Assistant custom card, "Tesla View", shows a real-time interactive 3D Tesla (vehicle state, not surroundings) — [Home Assistant community](https://community.home-assistant.io/t/custom-card-tesla-view-realtime-interactive-3d-tesla/1025234)
- University course projects (WPI RBE549, "P3") have students rebuild a Tesla-style dashboard visualisation from dash-camera video using off-the-shelf detection and depth networks, rendered offline — [WPI RBE549 project 3](https://rbe549.github.io/spring2025/proj/p3)
- Uber AVS is an open web toolkit for AV data visualisation — [Uber AVS blog](https://www.uber.com/us/en/blog/avs-autonomous-vehicle-visualization/)

### Inferences
- The WPI course project is the closest published precedent for the pipeline (monocular video to detected objects, lanes and depth to a rendered 3D scene with vehicle models). It runs offline on desktop hardware with a stack of separate networks, which confirms the approach but also that it is not a real-time embedded solution as published. I saw only the title/snippet, so the exact method list is unverified.
- Rerun, Foxglove and AVS are log/debug viewers: point clouds, boxes and plots for engineers. They are useful as a development harness for the perception side but are not styled, driver-facing displays, and none was found running on the comma device itself.
- Limits common to everything found: 2D overlays or desktop tools; no map-based world; no object memory outside the camera view; no use of an embedded NPU for extra perception.
- Because no prior art exists on this platform, the lowest-risk first version reuses data the device already publishes (model lane lines, leads, radar tracks, CAN objects, GPS pose) and adds map geometry, with no new neural network. Depth or detection networks can be added later once the NPU budget is measured.

### Gaps
- Foxglove with openpilot logs: nothing found.
- CARLA-based UIs and Autoware's RViz-style displays were not researched this session.
- I did not search sunnypilot, comma's own newer UI work, or Discord/Reddit, where a hobby 3D view could exist without a public repo. "Not found" here means not found in about five web searches, not proven absent.
- No project found that renders Gaussian splats or depth-reprojected camera imagery live on a comma device.
- The Oceanscape "Pulse HUD" clone mentioned in the user's own notes may already hold relevant code; it was outside this research scope and was not inspected.
