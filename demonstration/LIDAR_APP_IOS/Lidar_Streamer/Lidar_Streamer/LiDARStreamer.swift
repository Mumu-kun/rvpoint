import Combine
import Foundation
import ARKit
import SceneKit
import Network
import UIKit
import CoreMotion

final class LiDARStreamer: NSObject, ObservableObject, ARSessionDelegate {

    // Default Mac IP — the Python script prints it on startup
    @Published var host = "192.168.1.50"
    @Published var status = "Starting camera…"
    @Published var streaming = false
    @Published var framesSent = 0

    private let port: UInt16 = 9000
    private let minSendInterval: TimeInterval = 0.1   // ~10 packets/s

    private var arView: ARSCNView?
    private var connection: NWConnection?
    private var frameIndex: UInt32 = 0
    private var lastSend: TimeInterval = 0
    private var busy = false
    private let lock = NSLock()
    private let motionManager = CMMotionManager()

    // MARK: AR session

    func attach(to view: ARSCNView) {
        arView = view
        view.session.delegate = self

        guard ARWorldTrackingConfiguration.supportsFrameSemantics([.sceneDepth, .smoothedSceneDepth]) else {
            setStatus("LiDAR scene depth not supported on this device")
            return
        }
        let config = ARWorldTrackingConfiguration()
        config.frameSemantics = [.sceneDepth, .smoothedSceneDepth]
        view.session.run(config, options: [.resetTracking, .removeExistingAnchors])
        if motionManager.isDeviceMotionAvailable {
            motionManager.deviceMotionUpdateInterval = 0.02
            motionManager.startDeviceMotionUpdates()
        }
        setStatus("LiDAR running — enter Mac IP, then Start streaming")
    }

    // MARK: Networking

    func toggleStreaming() {
        streaming ? stopStreaming() : startStreaming()
    }

    private func startStreaming() {
        guard let nwPort = NWEndpoint.Port(rawValue: port) else { return }
        let conn = NWConnection(host: NWEndpoint.Host(host), port: nwPort, using: .tcp)
        connection = conn
        setStatus("Connecting to \(host):\(port)…")
        conn.stateUpdateHandler = { [weak self] state in
            guard let self else { return }
            switch state {
            case .ready:
                self.setStreaming(true)
                self.setStatus("Streaming to \(self.host):\(self.port)…")
            case .waiting(let error): self.setStatus("Waiting: \(error)")
            case .failed(let error):
                self.setStreaming(false)
                self.setStatus("Failed: \(error)")
            case .cancelled:
                self.setStreaming(false)
                self.setStatus("Disconnected")
            default: break
            }
        }
        conn.start(queue: DispatchQueue(label: "lidar.streamer", qos: .userInitiated))
    }

    private func stopStreaming() {
        connection?.cancel()
        connection = nil
        motionManager.stopDeviceMotionUpdates()
        setStreaming(false)
        setStatus("Stopped")
    }

    // MARK: ARSessionDelegate

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard streaming, connection != nil else { return }
        let now = Date().timeIntervalSince1970
        guard now - lastSend >= minSendInterval else { return }
        lock.lock(); let isBusy = busy; lock.unlock()
        guard !isBusy else { return }
        guard frame.sceneDepth != nil || frame.smoothedSceneDepth != nil else { return }
        lastSend = now

        let packet = buildPacket(frame: frame, index: frameIndex)
        frameIndex &+= 1
        send(packet)
    }

    func session(_ session: ARSession, didFailWithError error: Error) {
        setStatus("AR error: \(error.localizedDescription)")
    }

    func sessionWasInterrupted(_ session: ARSession) { setStatus("AR interrupted") }

    func sessionInterruptionEnded(_ session: ARSession) {
        if let view = arView { attach(to: view) }
    }

    // MARK: Packet: magic + header + depth(f32, m) + confidence(u8)

    private func buildPacket(frame: ARFrame, index: UInt32) -> Data {
        // The depth type is deliberately never named — avoids ARKit type-name mismatches
        guard let sceneDepth = frame.smoothedSceneDepth ?? frame.sceneDepth else { return Data() }
        let depthMap = sceneDepth.depthMap

        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

        let w = CVPixelBufferGetWidth(depthMap)
        let h = CVPixelBufferGetHeight(depthMap)
        let depthRow = CVPixelBufferGetBytesPerRow(depthMap) / MemoryLayout<Float32>.size
        let depthPtr = CVPixelBufferGetBaseAddress(depthMap)!.assumingMemoryBound(to: Float32.self)

        var depthArr = [Float32](repeating: 0, count: w * h)
        var conf = [UInt8](repeating: 0, count: w * h)
        for row in 0..<h {
            depthArr.replaceSubrange(row*w ..< row*w + w,
                                     with: UnsafeBufferPointer(start: depthPtr + row * depthRow, count: w))
        }

        if let confMap = sceneDepth.confidenceMap {
            CVPixelBufferLockBaseAddress(confMap, .readOnly)
            defer { CVPixelBufferUnlockBaseAddress(confMap, .readOnly) }
            let confRow = CVPixelBufferGetBytesPerRow(confMap)   // 1 byte per pixel
            let confPtr = CVPixelBufferGetBaseAddress(confMap)!.assumingMemoryBound(to: UInt8.self)
            for row in 0..<h {
                conf.replaceSubrange(row*w ..< row*w + w,
                                     with: UnsafeBufferPointer(start: confPtr + row * confRow, count: w))
            }
        }

        // Intrinsics rescaled from the capture image (1920x1440) to the depth map (256x192)
        let cam = frame.camera
        let sx = Float(w) / Float(cam.imageResolution.width)
        let sy = Float(h) / Float(cam.imageResolution.height)
        let fx = cam.intrinsics.columns.0.x * sx
        let fy = cam.intrinsics.columns.1.y * sy
        let cx = cam.intrinsics.columns.2.x * sx
        let cy = cam.intrinsics.columns.2.y * sy

        // Camera pose, row-major 4x4, with a Y/Z flip folded in so the Mac can use
        // the plain pinhole convention: x=(u-cx)*z/fx, y=(v-cy)*z/fy, z=depth.
        let t = cam.transform
        var pose: [Float32] = [
            t.columns.0.x, t.columns.1.x, t.columns.2.x, t.columns.3.x,
            t.columns.0.y, t.columns.1.y, t.columns.2.y, t.columns.3.y,
            t.columns.0.z, t.columns.1.z, t.columns.2.z, t.columns.3.z,
            t.columns.0.w, t.columns.1.w, t.columns.2.w, t.columns.3.w,
        ]
        for i in [1, 2, 5, 6, 9, 10, 13, 14] { pose[i] = -pose[i] }

        // Tracking state (0: notAvailable, 1: limited, 2: normal)
        let trackState: UInt32
        switch frame.camera.trackingState {
        case .notAvailable: trackState = 0
        case .limited: trackState = 1
        case .normal: trackState = 2
        @unknown default: trackState = 0
        }

        // CoreMotion readings (gyro, user accel, gravity)
        let motion = motionManager.deviceMotion
        let wx = Float(motion?.rotationRate.x ?? 0.0)
        let wy = Float(motion?.rotationRate.y ?? 0.0)
        let wz = Float(motion?.rotationRate.z ?? 0.0)
        let ax = Float(motion?.userAcceleration.x ?? 0.0)
        let ay = Float(motion?.userAcceleration.y ?? 0.0)
        let az = Float(motion?.userAcceleration.z ?? 0.0)
        let gx = Float((motion?.gravity.x ?? 0.0) * 9.80665)
        let gy = Float((motion?.gravity.y ?? 0.0) * 9.80665)
        let gz = Float((motion?.gravity.z ?? -1.0) * 9.80665)

        var data = Data(capacity: 136 + w * h * 5)
        data.append(Data("LDP2".utf8))
        data.appendUInt32(index)
        data.appendUInt32(UInt32(w))
        data.appendUInt32(UInt32(h))
        data.appendFloat(fx); data.appendFloat(fy); data.appendFloat(cx); data.appendFloat(cy)
        pose.forEach { data.appendFloat($0) }
        data.appendUInt32(trackState)
        data.appendFloat(wx); data.appendFloat(wy); data.appendFloat(wz)
        data.appendFloat(ax); data.appendFloat(ay); data.appendFloat(az)
        data.appendFloat(gx); data.appendFloat(gy); data.appendFloat(gz)
        depthArr.withUnsafeBytes { data.append(contentsOf: $0) }
        conf.withUnsafeBytes { data.append(contentsOf: $0) }
        return data
    }

    private func send(_ packet: Data) {
        guard let conn = connection else { return }
        lock.lock(); busy = true; lock.unlock()
        conn.send(content: packet, completion: .contentProcessed { [weak self] error in
            guard let self else { return }
            self.lock.lock(); self.busy = false; self.lock.unlock()
            if let error {
                self.setStatus("Send error: \(error)")
            } else {
                DispatchQueue.main.async { self.framesSent += 1 }
            }
        })
    }

    // MARK: thread-safe UI updates

    private func setStatus(_ text: String) {
        DispatchQueue.main.async { self.status = text }
    }

    private func setStreaming(_ value: Bool) {
        DispatchQueue.main.async {
            self.streaming = value
            UIApplication.shared.isIdleTimerDisabled = value   // keep screen awake
        }
    }
}

// Manual little-endian packing
private extension Data {
    mutating func appendUInt32(_ value: UInt32) {
        let v = value.littleEndian
        append(UInt8(truncatingIfNeeded: v))
        append(UInt8(truncatingIfNeeded: v >> 8))
        append(UInt8(truncatingIfNeeded: v >> 16))
        append(UInt8(truncatingIfNeeded: v >> 24))
    }
    mutating func appendFloat(_ value: Float32) {
        appendUInt32(value.bitPattern)
    }
}
