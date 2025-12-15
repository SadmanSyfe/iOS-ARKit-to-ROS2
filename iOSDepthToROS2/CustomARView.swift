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


let host: String = "XXX" //Host IP: ex. 192.XXX.XXX.XX, assumes port 9090
let websocket: WebSockets = WebSockets(ip: host)


private var activePayloads: [String : Payload] = [:]
let depthTopic = ImagePayload(topicName: "depth_raw")
let imageTopic = ImagePayload(topicName: "image_raw", encoding: "nv12") //Currently Not Using, Unstable and High Bandwith
let confidenceTopic = ImagePayload(topicName: "depth_confidence", encoding: "mono8")
let poseTfTopic = TransformStampedPayload(topicName: "pose_tf")



// Make CustomARView conform to ARSessionDelegate
class CustomARView: ARView, ARSessionDelegate {
    
    required init(frame frameRect: CGRect) {
        super.init(frame: frameRect)
        // Set the view's session delegate to itself
        session.delegate = self
        
        //TODO: ADD LOGIC TO CHECK WHAT TOPICS ARE ACTIVE OR NOT
        activePayloads["depth_raw"] = depthTopic
        //activePayloads["image_raw"] = imageTopic
        activePayloads["depth_confidence"] = confidenceTopic
        activePayloads["pose_tf"] = poseTfTopic

        
        // 1. Set the onConnect closure to call advertise
        websocket.onConnect = { [weak self] in
            // Use a slight delay to ensure the WebSocket is fully ready
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) {
                for payload in activePayloads{
                    websocket.advertiseTopic(payload: payload.value)
                }
            }
        }
    }
    
    dynamic required init?(coder decoder: NSCoder) {
        fatalError("init(coder:) has not been implmented")
    }
    
    private var lastPublishTime: TimeInterval = 0
    private let targetPublishInterval: TimeInterval = 1.0 / 10.0 // 10 FPS (0.1 seconds)
    
    
    // This delegate method is called automatically every time the session updates a frame
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        
        let currentTime = frame.timestamp
                if (currentTime - lastPublishTime) < targetPublishInterval {
                    return // Skip this frame if it's too soon
                }
                
        // Update the last publish time
        lastPublishTime = currentTime
        
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
        
        
        
        for payload in activePayloads{
            websocket.sendJSONString(jsonString: payload.value.getPayload())
        }
    }

    
    
    
}
