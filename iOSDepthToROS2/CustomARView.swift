//
//  CustomARView.swift
//  iOSDepthToROS2
//
//  Created by Ayan Syed on 12/11/25.
//

import ARKit
import RealityKit
import SwiftUI
import CoreVideo


let depthTopic = ImagePayload(topicName: "depth_raw")
let imageTopic = ImagePayload(topicName: "image_raw", encoding: "nv12") //Currently Not Using, Unstable and High Bandwith
let confidenceTopic = ImagePayload(topicName: "depth_confidence", encoding: "mono8")
let poseTfTopic = TransformStampedPayload(topicName: "pose_tf")
let cameraInfoTopic = CameraInfoPayload(topicName: "camera_info")



// Make CustomARView conform to ARSessionDelegate
class CustomARView: ARView, ARSessionDelegate {
    
    private var activePayloads: [String : Payload] = [:]
    private var websocket: WebSockets! // Will be initialized in init
    private var sessionTimeOffset: TimeInterval?
    private let defaults = UserDefaults.standard
    private var lastPublishTime: TimeInterval = 0
    private var targetPublishInterval: TimeInterval = 1.0 / 10.0 // 10 FPS (0.1 seconds)
    
    required init(frame frameRect: CGRect) {
        super.init(frame: frameRect)
        // Set the view's session delegate to itself
        session.delegate = self
        // 1. --- CONNECTION SETUP ---
        let rosHost = defaults.string(forKey: "ros_ip_address") ?? "XXX.XXX.X.XXX"
        self.websocket = WebSockets(ip: rosHost)
        
        // 2. --- TOPIC ACTIVATION ---
        
        // Load flags (Default values are defined in SettingsView, but defaults.bool(forKey:) returns false if key doesn't exist)
        let isDepthActive = defaults.bool(forKey: "topic_depth")
        let isPoseActive = defaults.bool(forKey: "topic_pose")
        let isImuActive = defaults.bool(forKey: "topic_imu")
        
        if isDepthActive {
            activePayloads["depth_raw"] = depthTopic
            activePayloads["depth_confidence"] = confidenceTopic
            activePayloads["camera_info"] = cameraInfoTopic
        }
        if isPoseActive {
            activePayloads["pose_tf"] = poseTfTopic
        }
        
        // 3. --- FPS SETUP ---
        let targetFPS = defaults.integer(forKey: "target_fps") // Will be 0 if unset, but Stepper starts at 1
        // Ensure you use a minimum FPS if the saved value is 0 or less
        let finalFPS = max(1, targetFPS)
        self.targetPublishInterval = 1.0 / TimeInterval(finalFPS)
        // 1. Set the onConnect closure to call advertise
        self.websocket.onConnect = { [weak self] in
            // Use a slight delay to ensure the WebSocket is fully ready
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) {
                for payload in self?.activePayloads ?? [:] {
                    self?.websocket.advertiseTopic(payload: payload.value)
                }
            }
        }
    }
    
    dynamic required init?(coder decoder: NSCoder) {
        fatalError("init(coder:) has not been implmented")
    }

    
    
    // This delegate method is called automatically every time the session updates a frame
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        
        let arFrameTime = frame.timestamp
                
                if (arFrameTime - lastPublishTime) < targetPublishInterval {
                    return // Skip this frame if it's too soon
                }
        
        // 1. Calculate and store the offset on the first run
        if sessionTimeOffset == nil {
            // UNIX time now - AR time now = offset
            sessionTimeOffset = Date().timeIntervalSince1970 - arFrameTime
        }
        
        // 2. Calculate the CORRECT UNIX Epoch time for the frame
        guard let offset = sessionTimeOffset else { return }
        
        let correctUnixEpochTime = offset + arFrameTime
        
        guard let sceneDepth = frame.sceneDepth else { return }
        if activePayloads["depth_raw"] != nil { //check if depth_raw is an activeTopic
            let (rawDepthData, width, height) = DataExtractor.extractRawDepthData(from: sceneDepth)
            depthTopic.updateData(data: rawDepthData, height: height, width: width)
        }
        if activePayloads["depth_confidence"] != nil { //check if depth_confidence is an activeTopic
            let (rawData, width, height) = DataExtractor.extractRawConfidenceData(from: sceneDepth)
            confidenceTopic.updateData(data: rawData, height: height, width: width)
        }
        
        let imagePixelBuffer = frame.capturedImage // CVPixelBuffer
        if activePayloads["image_raw"] != nil { //check if image_raw is an activeTopic
            imageTopic.stepMultiplier = 1
            let (rawData, width, height) = DataExtractor.extractRawImageData(from: imagePixelBuffer)
            //print("Height: \(height), Width: \(width)")
            imageTopic.updateData(data: rawData, height: height, width: width)
        }
        
        if activePayloads["pose_tf"] != nil { //check if pose_tf is an activeTopic
            let newTf = DataExtractor.getPoseTransform(frame: frame)
            poseTfTopic.updateTransform(newTransform: newTf)
        }
        
        if activePayloads["camera_info"] != nil { //check if pose_tf is an activeTopic
            let (width, height, K) = DataExtractor.getCameraInfo(frame: frame)
            cameraInfoTopic.updateResolution(height: height, width: width)
            cameraInfoTopic.updateK(newK: K)
        }
        
        for payload in activePayloads{
            websocket.sendJSONString(jsonString: payload.value.getPayload(frameTime: correctUnixEpochTime))
        }
        
        lastPublishTime = arFrameTime
    }

    
    
    
}
