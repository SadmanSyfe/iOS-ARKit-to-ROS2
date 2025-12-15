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



// Make CustomARView conform to ARSessionDelegate
class CustomARView: ARView, ARSessionDelegate {
    
    required init(frame frameRect: CGRect) {
        super.init(frame: frameRect)
        // Set the view's session delegate to itself
        session.delegate = self
        
        //TODO: ADD LOGIC TO CHECK WHAT TOPICS ARE ACTIVE OR NOT
        activePayloads["depth_raw"] = depthTopic
        
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
    

    // MARK: - ARSessionDelegate
    
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
            let (rawDepthData, width, height) = extractRawDepthData(from: sceneDepth)
            depthTopic.updateData(data: rawDepthData, height: height, width: width)
        }
        
        for payload in activePayloads{
            websocket.sendJSONString(jsonString: payload.value.getPayload())
        }
    }

    
    /// Converts an ARDepthData CVPixelBuffer into a raw Swift Data object.
    func extractRawDepthData(from depthData: ARDepthData) -> (data: Data, width: Int, height: Int) {
        
        // Get the CVPixelBuffer (The raw data container)
        let depthPixelBuffer = depthData.depthMap
        let width = CVPixelBufferGetWidth(depthPixelBuffer)
        let height = CVPixelBufferGetHeight(depthPixelBuffer)
        
        // 1. Lock the base address to access the raw memory
        CVPixelBufferLockBaseAddress(depthPixelBuffer, .readOnly)
        
        // Get a pointer to the start of the data
        guard let baseAddress = CVPixelBufferGetBaseAddress(depthPixelBuffer) else {
            CVPixelBufferUnlockBaseAddress(depthPixelBuffer, .readOnly)
            return (Data(), width, height)
        }
        
        // Calculate the total size of the data in bytes
        // For 32-bit float (4 bytes) and 1 channel:
        // Total Size = Width * Height * 4 Bytes
        let totalBytes = width * height * MemoryLayout<Float32>.size
        
        // 2. Create the Swift Data object by copying the bytes
        // This is the core conversion step.
        let data = Data(bytes: baseAddress, count: totalBytes)
        
        // 3. Unlock the base address to release the memory lock
        CVPixelBufferUnlockBaseAddress(depthPixelBuffer, .readOnly)
    
        
        return (data, width, height)
    }
    
}
