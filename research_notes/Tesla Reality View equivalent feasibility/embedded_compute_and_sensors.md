# Embedded compute, add-on sensors and software options for real-time perception and realistic 3D rendering on an RK3588S Android board (with optional comma device and add-on hardware), 2025-2026

Research date: 2026-11-23. Prices are USD unless marked. "Estimate" marks numbers that are not measured or not from a primary listing. Searches were limited (about 25 tool calls); several questions have no reliable public number and are listed under Gaps instead of being filled in.

## RK3588/RK3588S NPU: RKNN toolkit capabilities and limits, published benchmarks, and use from Android / WebView

### Takeaway
The NPU runs CNN detectors and segmenters comfortably (YOLOv8n/YOLO11n 60-74 FPS, yolov8n-seg about 61 FPS, single core, INT8, 640x640, inference only), but transformer models are a weak point: the only RK3588 figure found for Depth Anything V2 Small is about 696 ms per 518x518 frame (about 1.4 FPS, unquantized, 3 cores). No WebNN path to the NPU exists on Android; the realistic route from a WebView page is a native RKNN helper (JNI/service) feeding results to the page.

### Cited Findings

Rockchip rknn_model_zoo numbers as republished by Seeed (RK3588, single NPU core, model inference only, pre/post-processing excluded, maximum NPU frequency; page last updated 2026-11-01, toolkit version not stated):

| Model | Input | dtype | RK3588 FPS | RK3576 FPS |
|---|---|---|---|---|
| yolov5n | 640x640 | INT8 | 82.5 | 112.7 |
| yolov5s | 640x640 | INT8 | 48.4 | 57.5 |
| yolov5m | 640x640 | INT8 | 20.9 | 23.7 |
| yolov8n | 640x640 | INT8 | 73.5 | 90.2 |
| yolov8s | 640x640 | INT8 | 38.0 | 40.8 |
| yolov8m | 640x640 | INT8 | 16.2 | 16.7 |
| yolov10n | 640x640 | INT8 | 61.2 | 80.2 |
| yolo11n | 640x640 | INT8 | 60.0 | 77.9 |
| yolo11s | 640x640 | INT8 | 33.0 | 38.2 |
| yolo11m | 640x640 | INT8 | 12.7 | 14.6 |
| yolov8n-pose | 640x640 | INT8 | 55.9 | 66.8 |
| yolov5n-seg | 640x640 | INT8 | 69.3 | 88.3 |
| yolov8n-seg | 640x640 | INT8 | 60.8 | 71.1 |
| yolov8s-seg | 640x640 | INT8 | 28.9 | 30.8 |
| yolov8m-seg | 640x640 | INT8 | 12.6 | 12.7 |
| deeplab-v3-plus-mobilenet-v2 | 513x513 | INT8 | 34.0 | 39.4 |
| ppseg_lite | 512x512 | INT8 | 35.7 | 33.6 |
| yolo_world_v2s | 640x640 | INT8 | 22.1 | 22.3 |
| mobilesam_encoder_tiny | 448x448 | FP16 | 10.0 | 11.9 |
| clip_images (ViT) | 224x224 | FP16 | 6.5 | 6.7 |
| resnet50-v2 | 224x224 | INT8 | 110.1 | 99.0 |
| mobilenetv2 | 224x224 | INT8 | 450.7 | 467.0 |

— [Seeed SenseCraft: RK3576 and RK3588 Computer Vision Benchmarks](https://sensecraft.seeed.cc/ai-lab/en/tutorials/rk/benchmark/cv)

- The table above contains no monocular depth, occupancy or BEV model; the model zoo list shown there covers classification, detection, pose, segmentation, OCR, CLIP, SAM and audio only. — [Seeed SenseCraft benchmark page](https://sensecraft.seeed.cc/ai-lab/en/tutorials/rk/benchmark/cv)
- Depth Anything V2 Small on RK3588: a community RKNN conversion (fp32 ONNX, no quantization) reports a median 696.11 ms per image at 518x518 using 3 NPU cores (session medians 701.74 / 669.94 / 696.11 ms), model load about 250 ms; measured on a shared board with load about 1.8-2.0, so one author's measurement, not an independent benchmark. — [anguoyang/depth-anything-v2-small-rknn-rk3588 (Hugging Face)](https://huggingface.co/anguoyang/depth-anything-v2-small-rknn-rk3588)
- A Radxa forum user reported about 108 FPS for the rknn_model_zoo yolov8n demo on RK3588, and noted Python post-processing added roughly 30-40 ms per frame (anecdotal; conflicts with the 73.5 FPS single-core model zoo figure, probably because of multi-core/multi-thread use). — [Radxa forum: Use YoloV8 in RK3588 NPU](https://forum.radxa.com/t/use-yolov8-in-rk3588-npu/15838?page=7)
- ResNet18 INT8 224x224 on Orange Pi 5 Max (RK3588, RKNN runtime 1.6.0, driver 0.9.8, targeted at 3 cores): average 4.09 ms (244 FPS), all 26 operators on the NPU with no CPU fallback; NPU init 0.35 s. Post dated 2025-11-07. — [TinyComputers: RK3588 NPU Deep Dive](https://tinycomputers.io/posts/rockchip-rk3588-npu-benchmarks.html)
- Limits listed by the same author: conversion must run on an x86_64 host; INT8 is needed for best speed (float models run slower); fixed input shapes (dynamic shapes limited); some operators fall back to CPU; toolkit and runtime versions must match; transformer attention is "less optimized than CNNs". Power figures in that post (about 5-6 W total under AI load) are the author's estimates, not measurements. — [TinyComputers: RK3588 NPU Deep Dive](https://tinycomputers.io/posts/rockchip-rk3588-npu-benchmarks.html)
- The NPU is 3 cores (6 TOPS total). Core selection (`core_mask`: auto, 0, 1, 2, 0_1, 0_1_2) is set at inference time, not at export. Export can fail with "Unsupport NPU op" / "unsupported CPU op"; fixes are rewriting the ONNX graph or registering custom ops. — [RK-Transformers: RKNN Limitations](https://emapco.github.io/rk-transformers/user_guide/limitations.html); [RK-Transformers: NPU Core Configuration](https://emapco.github.io/rk-transformers/user_guide/npu_cores.html)
- RKNPU2 release notes state that RK3588 supports "multi-batch multi-core mode", and that `RKNN_LOG_LEVEL=4` prints per-layer MAC utilisation. — [rockchip-linux/rknpu2 README](https://github.com/rockchip-linux/rknpu2/blob/master/README.md)
- WebNN on Android: Chrome's team excluded Android from the WebNN origin trial (February 2026) "due to the implementation's immaturity on that platform"; only CPU inference works on Android, GPU and NPU support is planned but incomplete. The planned NPU route is TFLite's NNAPI delegate. — [blink-dev: WebNN origin trial, Android excluded](https://mid.mail-archive.com/blink-dev@chromium.org/msg15857.html); [webnn-docs PR 34](https://github.com/webmachinelearning/webnn-docs/pull/34/files); [Phoronix: Chrome 146 beta with WebNN origin trial](https://phoronix.com/news/chrome-146-beta)
- Ultralytics documents a direct YOLO to RKNN export path. — [Ultralytics: Rockchip RKNN integration](https://docs.ultralytics.com/integrations/rockchip-rknn/)
- Reference points for the same depth model elsewhere: Depth Anything V2 on a Qualcomm QCS8550 with TFLite is listed at 85.22 ms; a Qualcomm Dragonwing IQ-9075 runs it at over 11 FPS on its NPU. — [Qualcomm AI Hub: Depth-Anything-V2](https://huggingface.co/qualcomm/Depth-Anything-V2); [Hackster: depth estimation on Dragonwing boards](https://www.hackster.io/gigwegbe/depth-estimation-on-qualcomm-dragonwing-boards-4526e0)

### Inferences
- A practical real-time budget on this NPU is one small detector plus one small segmenter per camera stream at about 15-30 FPS each if spread across the three cores; four surround cameras at 640x640 would need batching or round-robin and would land near 15 FPS per camera for nano-size models (inference from the single-core table, not measured).
- The model zoo figures exclude pre/post-processing; the forum report of 30-40 ms Python post-processing shows that end-to-end rate depends heavily on doing resize/colour conversion in RGA and NMS in C++.
- Dense monocular depth with ViT backbones is not real-time on the RK3588 NPU as published (about 1.4 FPS for DA-V2 Small unquantized). The mobilesam (10 FPS, FP16) and CLIP image (6.5 FPS) rows support the general pattern that ViT workloads run roughly an order of magnitude slower than CNNs here. A CNN-based depth network or a lower-resolution, INT8 variant would have to be tested locally.
- Because RKNN is not an NNAPI backend that Chrome can reach, and WebNN on Android is CPU-only and outside the origin trial, a WebView page cannot drive the NPU. The workable design is a native Android service linking librknnrt via JNI that pushes detections/depth to the page over a localhost WebSocket or a `WebMessagePort`/JavaScript interface; WASM in the WebView would run on the CPU only.
- "Multi-batch multi-core" wording suggests the three cores speed up throughput (several inferences in parallel) more than they shorten one inference.

### Gaps
- No published RK3588 benchmark was found for MiDaS, Depth Anything V2 Base, or any occupancy/BEV network (BEVFormer, BEVDet, FlashOcc, etc.). Those models rely on ops (deformable attention, grid_sample, voxel pooling) whose RKNN support could not be confirmed in this pass.
- No measured single-core versus three-core speedup for one model was found.
- The official supported-operator list and the current rknn-toolkit2 version were not read directly; quantization options beyond INT8/FP16 (for example mixed/hybrid quantization) are not sourced here.
- No source was found documenting RKNN use on Android specifically (the runtime ships Android libraries, but this was not verified in this pass) or its behaviour on RK3588S versus RK3588 (assumed identical NPU; unverified).
- PhotonVision publishes an Orange Pi 5 INT8 comparison of YOLOv5/v8/v11 on COCO, but the units of its table could not be confirmed, so no numbers are quoted. [PhotonVision RKNN benchmarks](https://docs.photonvision.org/en/latest/docs/benchmarks/rknn-model-benchmarks.html)

## Mali-G610 GPU: WebGL2 vs WebGPU in Android WebView, budgets, video textures, and 3D Gaussian splatting on mobile GPUs

### Takeaway
WebGPU is on by default in Chrome for Android 12+ on ARM (Mali) and Qualcomm GPUs since Chrome 121, so a Mali-G610 on Android 13 is nominally in scope, but nothing found confirms it for this GPU in a WebView. Published splat numbers on stronger phone GPUs (20-38 FPS WebGL2 for 1-4M splats on an iPhone 13 Pro Max; 300-580 ms per frame for a compute-based renderer on Snapdragon 8 Gen 2) indicate that photoreal splat scenes at car-display resolution are marginal on a Mali-G610 MP4.

### Cited Findings
- WebGPU is "enabled by default in Chrome 121 on devices running Android 12 and greater powered by Qualcomm and ARM GPUs". — [Chrome for Developers: What's new in WebGPU (Chrome 121)](https://developer.chrome.com/blog/new-in-webgpu-121)
- The Intent to Ship said WebGPU would initially be enabled for Adreno and Mali GPUs on Android 12+, and lists Android WebView among target platforms. — [blink-dev: Intent to Ship WebGPU on Android](https://groups.google.com/a/chromium.org/g/blink-dev/c/YFWuDlCKTP4)
- Chrome 127 added experimental WebGPU support on OpenGL ES (compatibility mode) on Android, and a separate Intent to Ship covers WebGPU Compatibility mode. — [Chrome for Developers: What's new in WebGPU (Chrome 127)](https://developer.chrome.com/blog/new-in-webgpu-127); [blink-dev: Intent to Ship WebGPU Compatibility mode](https://groups.google.com/a/chromium.org/g/blink-dev/c/N3RlLGCOTJ4)
- Mali-G610 MP4 aggregate scores (phone SoCs, user-submitted, not RK3588 boards): 3DMark Wild Life Unlimited median 4483 (range 4259-4801); Wild Life Extreme Unlimited median 1202; Steel Nomad Light 523; GFXBench Aztec Ruins High Tier offscreen (4K) about 10 FPS; Manhattan offscreen median 106 FPS; T-Rex offscreen median 166 FPS. — [Notebookcheck: ARM Mali-G610 MP4 Benchmarks and Specs](https://www.notebookcheck.net/ARM-Mali-G610-MP4-Benchmarks-and-Specs.814022.0.html)
- An RK3588 evaluation board on Android 12 has a Geekbench 6 GPU compute score of 16994 (API not stated). — [Geekbench Browser entry](https://browser.geekbench.com/v6/compute/5028118)
- 3DGS in the browser, PlayCanvas SuperSplat (post dated 2026-06-03), iPhone 13 Pro Max at 642x1126:

| Splats | 1M | 2M | 3M | 4M |
|---|---|---|---|---|
| WebGL 2 FPS | 38.1 | 35.5 | 25.5 | 20.4 |
| WebGPU FPS | 77.6 | 73.2 | 52.7 | 42.4 |

On an Apple M4 Max at 1298x962: WebGL2 137 FPS at 1M falling to 48 FPS at 10M; WebGPU 139 FPS at 1M and 124 FPS at 10M. No Android or Mali device was tested. — [PlayCanvas blog: New in SuperSplat, WebGPU and streaming](https://blog.playcanvas.com/new-in-supersplat-webgpu-and-streaming-bring-huge-performance-wins)
- Vulkan-compute 3DGS on Snapdragon 8 Gen 2 (Adreno, 12 GB): full-pipeline latency 299 ms (Train, 980x545, 741k Gaussians), 314 ms (Truck, 979x546, 1.69M), 584 ms (Playroom, 1264x832, 1.49M) after optimisation; sorting dominates (267-506 ms) while the render stage is 20.9-59.5 ms. The full baseline "could not be run on older platforms" (Adreno 540, Mali-G57 MC2), where only sorting was tested. The paper is internally inconsistent about its headline speedup (1.7x abstract, 1.6x conclusion, 1.27x table maximum). — [Optimizing 3D Gaussian Splattering for Mobile GPUs (arXiv 2511.16298)](https://arxiv.org/html/2511.16298v1)
- Mobile-GS reports 116 FPS on a Snapdragon 8 Gen 3 device (resolution and scene not captured in this pass). — [Mobile-GS (GitHub)](https://github.com/catid/Mobile-GS)
- The WebSplatter paper argues real-time rendering of large 3DGS scenes "remains impractical under the WebGL API" and presents a WebGPU renderer for desktop and mobile. — [WebSplatter (arXiv 2602.03207)](https://arxiv.org/html/2602.03207v1)
- Video into textures: the proposed `WEBGL_webcodecs_video_frame` extension is marked "DO NOT IMPLEMENT"; the WebGPU zero-copy VideoFrame import shipped first on ChromeOS, macOS and Windows only; a Chromium media-dev thread discusses whether MediaCodec-decoded frames reach SharedImages through a deep copy on Android and notes frames can only be used in order there. — [Khronos WEBGL_webcodecs_video_frame proposal](https://registry.khronos.org/webgl/extensions/proposals/WEBGL_webcodecs_video_frame/); [blink-dev: Intent to Ship WebGPU WebCodecs integration](https://groups.google.com/a/chromium.org/g/blink-dev/c/QLCfazM8XLQ); [media-dev: SharedImage on Android](https://groups.google.com/a/chromium.org/g/media-dev/c/fPsc9zmkmLk)
- H.265 in WebRTC shipped for Android WebView in Chrome 136 (a different API from WebCodecs). — [blink-dev: Intent to Ship H265 in WebRTC](https://groups.google.com/a/chromium.org/g/blink-dev/c/3h8lL8a377c)
- A Mozilla bug report notes Chrome on Android handles video-as-texture uploads correctly. — [Mozilla bug 1884282](https://bugzilla.mozilla.org/show_bug.cgi?id=1884282)

### Inferences
- WebGPU availability on the board depends on the installed WebView build (the Android 13 system WebView on an AOSP Rockchip image may be old and not Play-updated) and on the vendor Mali Vulkan driver passing Chrome's blocklist. It must be feature-detected with `navigator.gpu.requestAdapter()`; WebGL2 should be the baseline design target.
- The Mali-G610 MP4 scores above put it well under the phone GPUs used in the splat papers, so a WebGL2 splat viewer should be planned for well under 1M splats and reduced render resolution (estimate; no Mali-G610 splat measurement exists in the sources found). The CPU-side depth sort is the dominant cost in the measured mobile pipeline, which also competes with perception for the A76 cores.
- A hybrid scene (textured meshes for road/vehicles, optional small splat patches) fits this GPU far better than a full splat environment. Tile-based Mali GPUs are sensitive to overdraw, which is exactly what alpha-blended splats produce.
- Using a `<video>` element or WebCodecs `VideoFrame` with `texImage2D` is the supported path for camera video in WebGL; whether it is zero-copy in this WebView is undocumented and must be measured.

### Gaps
- No Mali-G610 or RK3588-specific measurement of WebGL2/WebGPU frame rate, triangle or draw-call budget, or any 3DGS viewer was found. Triangle/draw-call budgets in this note are therefore absent rather than estimated.
- No confirmation of WebGPU working in Android System WebView on an RK3588 board (versus Chrome on a Mali phone).
- No benchmark of WebCodecs H.264/H.265 hardware decode into WebGL textures on Android WebView; H.265 WebCodecs support on this platform is unconfirmed.
- Arm Mali developer documentation (shader/overdraw guidance) was not consulted in this pass.

## Add-on sensors for 360-degree coverage: cameras and capture hardware, low-cost radars/lidars, tapping factory surround-view

### Takeaway
Entry 360-degree lidars are now about $750-$1,000 (Hesai JT16 about $745, Livox Mid-360 $979 list) and TI single-chip radar EVMs are about $180-$400, all with ROS/Linux drivers rather than Android support. GMSL deserialiser boards exist off the shelf for Raspberry Pi 5 and Jetson ($88-$270) but none was found that targets RK3588 out of the box, so camera capture on the Rockchip board is a driver/device-tree project.

### Cited Findings
- Livox Mid-360 list price is $979, with a note that tariffs can change it; a Mid-360L variant (half point rate) was announced in September 2026 without a published price. Marketplace prices range from $734 to $1,247 (eBay), and a Unitree-branded listing is $4,275. — [DroneXL: Livox Mid-360L](https://dronexl.co/2026/09/21/livox-mid-360l-lidar-half-point-rate-robots/); [DJI Store: Livox Mid-360](https://store.dji.com/product/livox-mid-360); [eBay listing](https://www.ebay.com/itm/156900478912); [Stemfinity: Unitree Livox MID360](https://stemfinity.com/collections/unitree/products/livox-mid360-lidar)
- Hesai JT16 (mini 3D lidar, JT series announced at CES 2025 with 360 x 187 degree hyper-hemispherical FOV on the series) is listed at $745 (backordered) at one retailer and at EUR 677.55 sale / EUR 812.90 regular at another. — [Aifitlab: Hesai JT16](https://aifitlab.com/products/hesai-jt16-lidar); [OpenELAB: Hesai](https://openelab.io/collections/brands-hesai); [Hesai press release via Nasdaq](https://www.nasdaq.com/press-release/hesai-unveils-mini-hyper-hemispherical-3d-lidar-series-robotics-ces-2025-01-07)
- TI IWR6843AOPEVM (60 GHz antenna-on-package radar EVM): $178.80 at DigiKey, $198.16 at Mouser. TI AWR1843BOOST (76-81 GHz automotive radar EVM): $358.80 at DigiKey, $397.69 at Mouser. — [DigiKey: IWR6843AOPEVM](https://www.digikey.com/en/products/detail/texas-instruments/AWR6843AOPEVM/14544363); [Mouser: AWR1843BOOST](https://www.mouser.com/ti-awr1843boost-eval-module/); [DigiKey: AWR1843BOOST](https://www.digikey.de/en/products/detail/texas-instruments/AWR1843BOOST/10445300)
- TI publishes a ROS/ROS 2 mmWave driver node that configures the EVM and publishes `PointCloud2`; TI says it is tested with the IWR6843ISK and works with the IWR6843AOPEVM running the mmWave SDK 3.5 out-of-box demo. A community ROS 2 package reports testing on AWR1843AOP. — [TI mmWave ROS Driver Users Guide](https://dev.ti.com/tirex/explore/content/mmwave_industrial_toolbox_4_7_0/labs/robotics/ros_driver/docs/TI_mmWave_ROS_Driver_Users_Guide.html); [TI Robotics SDK radar node (ROS 2)](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/latest/docs/source/docs/radar_driver_node_ros2.html); [Plaba/ti_mmwave_ros2](https://www.github.com/Plaba/ti_mmwave_ros2)
- Waveshare 4-channel GMSL adapter (MAX9296 deserialiser, two 4-lane MIPI CSI 22-pin outputs, FAKRA Z connectors with 12 V power-over-coax, 12 V input): $269.99, stated compatible with Raspberry Pi 5. The 2-channel MAX9296A module is $87.95-$89.99 and targets Raspberry Pi 5 and Jetson Orin Nano/NX. — [Waveshare GMSL Module Board 4CH A](https://www.waveshare.com/product/ai/cameras/others/gmsl-module-board-4ch-a.htm); [Waveshare MAX9296 GMSL deser module](https://www.waveshare.com/product/raspberry-pi/cameras/gmsl-cameras/max9296-gmsl-deser-module.htm)
- The automotive RK3588M is advertised with up to 16 camera inputs; a commenter notes the RK3588 datasheet lists 6 MIPI CSI-2 inputs. — [CNX Software: RK3588M](https://cnx-software.com/?p=110365)
- A Rockchip board with four AHD camera inputs exists on RK3568 (Boardcon EM3568-AV CAM, 4x BNC through an analog-to-MIPI-CSI chip), showing the AHD-to-CSI route is established in the Rockchip ecosystem. — [CNX Software: RK3568 SBC with four AHD cameras](https://cnx-software.com/?p=117899)
- Rockchip publishes a UVC host troubleshooting guide (Chinese) covering preview latency analysis with usbmon and v4l2-ctl. — [Rockchip Trouble Shooting Linux USB Host UVC](https://yvcherl.github.io/newpdf/Common/USB/Rockchip_Trouble_Shooting_Linux_USB_Host_UVC_CN.pdf)
- UVC host-side latency reference (Jetson TX1, 1280x1080 at 30 FPS): about 105 ms with GStreamer, about 168 ms with OpenCV. — [NVIDIA forum: USB camera high latency on TX1](https://forums.developer.nvidia.com/t/usb-camera-high-latency-on-tx1/55532)
- A forum report shows a UVC camera dropping from a promised 120 FPS to 15 FPS in MJPG mode with only about 3 Mb/s flowing, illustrating that format/driver behaviour can dominate nominal bandwidth. — [NVIDIA forum: V4L2 MJPG low framerate](https://forums.developer.nvidia.com/t/v4l2-mjpg-low-framerate/300048)
- e-con Systems describes latency as more critical in surround-view than in most other multi-camera applications. — [Edge AI and Vision Alliance / e-con Systems: choosing a surround-view camera](https://www.edge-ai-vision.com/2024/12/how-to-choose-the-right-surround-view-camera-for-vehicles/)

### Inferences
- The GMSL boards above output standard MIPI CSI-2, so electrically they can feed an RK3588S CSI port, but a MAX9296 kernel driver, device-tree nodes and sensor ISP tuning for the specific camera would have to be brought up on the vendor Android kernel. That is days-to-weeks of BSP work and depends on which CSI connectors the specific RK3588S board exposes (RK3588S exposes fewer CSI ports than RK3588; exact count is board-specific and was not verified).
- USB UVC is the lowest-effort route on Android (Camera2/UVC libraries, no kernel work) but four 1080p streams need MJPEG or H.264 cameras and separate USB controllers, and expected host-side latency is on the order of 100 ms or more before any processing (inferred from the TX1 figures; not measured on RK3588).
- Lidar and radar vendors ship Linux/ROS drivers. On an Android board the sensors would need either a Linux container/chroot, a port of the UDP/serial protocol into an Android service (Livox and Hesai use Ethernet UDP; TI EVMs use USB serial), or a separate Linux computer. None lists Android support in the sources found.
- For a rendered "reality view", a single roof-level Mid-360 or JT16 gives geometry around the car that cameras alone cannot provide on this NPU; cost about $750-$1,000 plus a 12 V supply and mounting. This is a hardware shortcut around the missing real-time depth/occupancy capability noted above.

### Gaps
- No pricing found for RoboSense Airy or other RoboSense entry models.
- No measured latency or pricing found for AHD-to-USB capture devices, or for four simultaneous UVC cameras on RK3588.
- No reliable source found on tapping a factory surround-view feed (LVDS / GMSL / FPD-Link III), including which serialiser family the Fisker Ocean uses. Tapping requires a matching deserialiser plus knowledge of the link configuration; this is unverified and should be treated as high-risk.
- One low-quality page (techyorker.com) claims RK3588 UVC 1080p at 28-32 FPS and 28-35 ms latency on RK3566; not verifiable, not used.
- Lidar specifications (range, point rate, power draw) were not collected in this pass.

## Alternative compute: Jetson Orin Nano / Orin NX, mini PCs, comma four or phone, and 12 V power/thermal constraints

### Takeaway
Jetson prices rose sharply in 2026 (Orin Nano Super dev kit $249 to $399; Orin NX 16GB module $599 to $999), and the available depth numbers on Orin are modest (about 10 FPS on Orin NX and 20 FPS on AGX Orin for a ViT-S depth model on four 480x300 images), so a Jetson is a clear upgrade over the RK3588 NPU for transformer depth but not a free lunch. The comma four is a Snapdragon 845 device already loaded with driving models and is not a credible rendering host.

### Cited Findings
- Jetson price changes reported 2026-07-22 (previous to new): Orin Nano Super Developer Kit $249 to $399; Orin Nano 8GB module (1KU+) $299 to $399; Orin Nano 4GB module $199 to $349; Orin NX 8GB module $399 to $649; Orin NX 16GB module $599 to $999; AGX Orin Developer Kit $1,999 to $3,499; AGX Orin 32GB module $899 to $1,799. No explanation was announced; the article links it to high LPDDR4 prices without confirming. — [CNX Software: NVIDIA increases the price of Jetson modules and devkits by up to 101%](https://cnx-software.com/2026/07/22/nvidia-increases-the-price-of-jetson-modules-and-devkits-by-up-to-101)
- Orin Nano Super: 67 INT8 TOPS, a new 25 W mode giving 30-70% more performance than the 15 W mode. Orin NX: up to 157 TOPS (sparse; 78 dense on 16GB in Super mode), power configurable 10-40 W. The dev kit carrier can supply up to 45 W. — [NVIDIA blog: Jetson Orin Nano Super](https://blogs.nvidia.com/blog/jetson-generative-ai-supercomputer); [Hackster: Jetson Orin Nano Super](https://www.hackster.io/news/nvidia-unveils-the-jetson-orin-nano-super-delivers-a-1-7x-gen-ai-speed-boost-for-all-3602c2e98c40); [NVIDIA Jetson Orin page](https://www.nvidia.com/en-eu/autonomous-machines/embedded-systems/jetson-orin/); [JetsonHacks: Orin Nano Super dev kit](https://jetsonhacks.com/2024/12/17/jetson-orin-nano-super-developer-kit/)
- Depth on Jetson: a fine-tuned Depth Anything V2 ViT-S exported to TensorRT, batch of four 480x300 images, runs at 19.9 FPS on Jetson AGX Orin (JetPack 6.1) and 10.1 FPS on Orin NX. — [arXiv 2603.01999: Learning Vision-Based Omnidirectional Navigation](https://arxiv.org/pdf/2603.01999)
- Depth Anything 3 Small alone reaches 21.9 FPS using 5.1 GB on Orin Nano. — [arXiv 2603.28890: Bootstrap Perception Under Hardware Depth Failure](https://arxiv.org/pdf/2603.28890)
- On an RTX A2000 laptop GPU, Depth Anything V2 Small at 518 runs 127 FPS in FP16 and 41.3 FPS in INT8 under trtexec (INT8 slower than FP16 for this model). — [NVIDIA forum: INT8 worse than FP16 for MiDaS DPT Hybrid on Thor](https://forums.developer.nvidia.com/t/int8-throughput-and-latency-worse-than-fp16-for-midas-dpt-hybrid-model-on-thor/354915)
- A 2026 benchmark paper compares monocular and stereo depth estimators on Jetson AGX Orin (not read in this pass). — [arXiv 2610.27199: Which Depth Estimator for Jetson AGX Orin?](https://arxiv.org/pdf/2610.27199)
- comma four: $799 (was $999; $749 with trade-in), Qualcomm Snapdragon 845 "MAX" with a cooling system, 1.9-inch 300 PPI OLED, two wide cameras plus a narrow camera, 128 GB storage, LTE and Wi-Fi, CAN FD, OBD-C and a USB 3.1 Gen 2 port. GPU, RAM and power draw are not listed. — [comma.ai shop: comma four](https://comma.ai/shop/comma-four)
- Phone-class reference for splat rendering: Snapdragon 8 Gen 3 at 116 FPS (Mobile-GS); iPhone 13 Pro Max at 42-78 FPS WebGPU for 1-4M splats. — [Mobile-GS (GitHub)](https://github.com/catid/Mobile-GS); [PlayCanvas SuperSplat post](https://blog.playcanvas.com/new-in-supersplat-webgpu-and-streaming-bring-huge-performance-wins)
- Comparison table from one blogger (not independently verified, prices dated 2025-11): Orange Pi 5 Max about $150-180; Jetson Orin Nano listed at $499 with 40 TOPS. — [TinyComputers: RK3588 NPU Deep Dive](https://tinycomputers.io/posts/rockchip-rk3588-npu-benchmarks.html)

### Inferences
- The comma four has a 1.9-inch screen and a seven-year-old SoC whose GPU/DSP is already occupied by the driving and monitoring models; this project's own notes record that extra jobs on the device ran it out of RAM and killed modeld mid-drive. It should be treated as a data source (model outputs, lead/lane/pose over the network) and not as a renderer or extra perception host.
- A current flagship phone has a GPU several times stronger than the Mali-G610 MP4 and could render the scene and cast it, but adds a wireless/USB display hop and thermal throttling on a dashboard; no measurement was found for that arrangement.
- An Orin Nano Super (25 W peak) or Orin NX (up to 40 W) draws about 2-3.5 A at 12 V, well within an accessory/cigarette-lighter circuit (commonly fused at 10-15 A; general knowledge, not sourced here). The practical constraints are a clean DC-DC supply that survives cranking/load-dump transients, switched-ignition shutdown, and cabin heat (parked-car interior temperatures exceed the commercial 0-35 C ambient range many dev kits are rated for; estimate).
- Cost of the Jetson route in late 2026: about $399 for an Orin Nano Super dev kit, or roughly $1,000+ for an Orin NX 16GB module plus a carrier board (carrier price not sourced). The Jetson would do depth/occupancy and could also render, sending video or scene data to the RK3588S display.

### Gaps
- No measured Jetson Orin Nano number for Depth Anything V2 Small at 518 with TensorRT, and no occupancy-network or 3DGS rendering benchmark on Orin Nano/NX was found.
- No pricing or measurements for mini PCs with discrete or strong integrated GPUs (for example Ryzen AI or Intel Arc based) were collected.
- No sourced automotive 12 V budget figures (fuse ratings, accessory-socket limits, EV DC-DC capacity) or thermal derating data were collected; the statements above are estimates.
- RK3588S board power draw under combined NPU + GPU + decode load was not found as a measurement (the 5-6 W figure cited earlier is an estimate by its author).

## Reusable software stacks and licences

### Takeaway
Autoware is Apache-2.0 and is the most reusable open perception stack, but it assumes ROS 2 on Linux with CUDA/TensorRT for its heavier perception nodes; on the Rockchip Android board the realistic reuse is at the level of models, message definitions and algorithms, with RKNN for inference and Android MediaCodec/WebView for video and rendering.

### Cited Findings
- Autoware is licensed under Apache License 2.0, and contributions are accepted under the same licence. — [Autoware documentation: License](https://docs.autoware.org/main/contributing/license/)
- `autoware_core_perception` is an Apache-2.0 meta-package that includes modules such as the euclidean cluster object detector and ground filter; Autoware perception message packages are also Apache-2.0. — [ROS Index: autoware_core_perception](https://index.ros.org/p/autoware_core_perception/); [ROS Index: autoware_perception_msgs](https://index.ros.org/p/autoware_perception_msgs/)
- TI's Robotics SDK provides ROS 2 radar driver nodes publishing `PointCloud2`. — [TI Robotics SDK radar node (ROS 2)](https://software-dl.ti.com/jacinto7/esd/robotics-sdk/latest/docs/source/docs/radar_driver_node_ros2.html)
- Ultralytics supports exporting YOLO models to RKNN. — [Ultralytics: Rockchip RKNN integration](https://docs.ultralytics.com/integrations/rockchip-rknn/)
- NVIDIA TAO Deploy documents building TensorRT engines for monocular depth (Depth Anything V2 based, 518 input in the examples), and NVIDIA publishes an "nvdepthanythingv2" model on NGC. — [NVIDIA TAO Deploy: Monocular Depth](https://docs.nvidia.com/tao/tao-toolkit/text/tao_deploy/mono_depth.html); [NGC: nvdepthanythingv2](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/tao/models/nvdepthanythingv2)
- Rockchip's model zoo (detection, segmentation, pose, OCR, SAM, CLIP demos) is public on GitHub. — [airockchip/rknn_model_zoo](https://github.com/airockchip/rknn_model_zoo)
- Web renderers for Gaussian splats that could be embedded in a WebView page: PlayCanvas SuperSplat (WebGL2 and WebGPU), a W3C basic WebGPU splat example, and community WebGPU/WebGL viewers. — [PlayCanvas SuperSplat post](https://blog.playcanvas.com/new-in-supersplat-webgpu-and-streaming-bring-huge-performance-wins); [W3C: Basic example for Gaussian Splats in WebGPU](https://www.w3.org/2026/splat/); [jeantimex/splat](https://github.com/jeantimex/splat)

### Inferences
- Ultralytics YOLO code and weights are AGPL-3.0 unless a commercial licence is bought (general knowledge; not sourced in this pass). For a personal in-car project this is acceptable, but it matters if the work is ever distributed; YOLOX/PP-YOLOE entries in the Rockchip zoo are the permissive alternatives usually chosen for that reason.
- NVIDIA DriveWorks is tied to the DRIVE platform and its developer programme, and Isaac ROS packages (nvblox, etc.) target Jetson/CUDA; neither runs on Rockchip. They are only reusable if a Jetson is added (unsourced here; see Gaps).
- On the Rockchip side the efficient Linux pipeline is GStreamer with the `mpp` (rkmpp) decoder/encoder elements and RGA for resize/colour conversion; on Android the equivalent is MediaCodec (which fronts the same Rockchip hardware codec) decoding to a `SurfaceTexture`. A WebView page can only consume that through a `<video>`/WebRTC/WebCodecs stream, so a native overlay (SurfaceView behind a transparent WebView) is the low-latency alternative to routing camera video through the page (design inference, not from a source).
- ROS 2 on Android is not a mainstream supported target; a lighter transport (plain UDP/WebSocket, or the cereal/msgq-style messaging already used by openpilot) is a more proportionate choice for a single-vehicle system than porting ROS 2.

### Gaps
- No sources were retrieved for: Isaac ROS / nvblox licence and requirements, DriveWorks availability, ROS 2 on Android status, the Ultralytics licence text, GStreamer rkmpp/RGA documentation, or Android MediaCodec-to-WebGL paths. Everything on those topics above is inference from general knowledge and should be verified before the report relies on it.
- Depth Anything V2 licence differences (Small is Apache-2.0; Base/Large are CC-BY-NC-4.0 per the upstream repository as generally reported) were not verified in this pass.

## End-to-end latency budgets for camera-to-display and what users accept

### Takeaway
The only hard numbers found are vendor reference designs: TI reports about 60-65 ms glass-to-glass for a one-camera, one-display mirror-replacement pipeline without deep learning, and a hobby-grade UVC path measures 105-168 ms. No numeric limit from ISO 16505 / UN R46 or a human-factors threshold could be confirmed from accessible sources.

### Cited Findings
- TI's AM62A camera-mirror reference: estimated camera-to-display latency about 65 ms, described as adequate for a typical camera mirror system; the demonstration achieves 60 ms glass-to-glass "not including deep learning related processing". — [TI SPRADC4 (June 2023)](https://ti.com/lit/spradc4)
- ISO 16505 treats time behaviour (frame rate, image formation time, overall system latency) as a performance domain for camera monitor systems; UN R46 follows ISO 16505:2015 and adds requirements such as flicker and image formation time. The numeric values were not visible in the accessible previews. — [ISO 16505:2019 catalogue entry](https://standards.iteh.ai/catalog/standards/iso/fd6c00ff-d934-46a8-b135-bf15a145fe05/iso-16505-2019); [TUV Rheinland: Camera Monitor Systems](https://www.tuv.com/world/en/camera-monitor-systems-for-the-automotive-industry.html)
- UVC camera through a Jetson TX1 at 1280x1080/30 FPS: about 105 ms (GStreamer) and about 168 ms (OpenCV). — [NVIDIA forum: USB camera high latency on TX1](https://forums.developer.nvidia.com/t/usb-camera-high-latency-on-tx1/55532)
- PhotonVision documents a method for characterising end-to-end camera latency on RK3588-class coprocessors. — [PhotonVision: Latency Characterization](https://docs.photonvision.org/en/latest/docs/contributing/design-descriptions/e2e-latency.html)
- A 2026 NHTSA report (DOT HS 813 776) compared driver response with camera-based rear visibility systems against mirrors and found faster response with the CMS in one of six comparisons; it does not set a latency threshold. — [NHTSA / ROSA P: dot_93528](https://rosap.ntl.bts.gov/view/dot/93528/dot_93528_DS1.pdf)
- Frame-time contributions measurable from this note's other sections: a 30 FPS camera adds up to 33 ms of frame interval; YOLO-class inference on the RK3588 NPU is 13-17 ms per frame per core at nano size (60-74 FPS); Depth Anything V2 Small is about 696 ms. — [Seeed SenseCraft benchmark page](https://sensecraft.seeed.cc/ai-lab/en/tutorials/rk/benchmark/cv); [anguoyang DA-V2 RKNN model card](https://huggingface.co/anguoyang/depth-anything-v2-small-rknn-rk3588)

### Inferences
- A rendered surroundings view is a visualisation of perception outputs, not a mirror replacement, so the relevant requirement is perceived smoothness and plausibility more than a regulatory glass-to-glass limit. A plausible pipeline on this hardware (30 FPS capture about 33 ms, decode/copy 10-30 ms, detection 15-30 ms, tracking and scene update under 5 ms, render and vsync 16-33 ms) lands at roughly 100-150 ms object latency (estimate assembled from the cited components; not measured).
- Latency in an object-based view can be hidden by rendering at display rate from a tracker that extrapolates object and ego motion (ego motion is available at high rate from the comma device), so perception can run at 10-15 FPS while the picture stays at 30-60 FPS. This is the main reason the slow depth model is less damaging than its raw frame rate suggests, provided depth is used for slowly-changing static geometry only.
- Anything used for manoeuvring decisions (a live camera surround view while parking) should be held to the mirror-replacement class of latency, about 60-100 ms, which favours a native video path over routing camera frames through the WebView.

### Gaps
- The numeric latency limit in ISO 16505 / UN R46 (commonly quoted as under 200 ms) could not be confirmed from an accessible primary source.
- No published glass-to-glass figure for production surround-view (AVM) systems, AR HUDs, or Tesla's visualisation was found, and no human-factors study giving a perceptual acceptability threshold for a rendered surroundings view.
- No measured camera-to-display latency on an RK3588 Android board (UVC or CSI, WebView or native) was found.
