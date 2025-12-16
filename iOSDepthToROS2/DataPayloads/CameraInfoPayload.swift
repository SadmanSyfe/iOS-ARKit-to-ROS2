//
//  ImagePayload.swift
//  iOSDepthToROS2
//
//  Created by Ayan Syed on 12/14/25.
//

import Foundation
import simd

class CameraInfoPayload: Payload{
    let isBigEndian: Int = 0
    let topicType: String = "depth/"
    let msgType: String = "CameraInfo"
    
    var distortion_model: String = "plumb_bob"
    var height: Int = 0
    var width: Int = 0
    var K: [Double] = []
    
    
    init(topicName: String){
        super.init(topicField: (self.topicType + topicName), msgType: self.msgType)
        print("Created Image Topic Class: " + self.topic + " with Type: " + self.type)
    }

    
    
    func updateResolution(height: Int, width: Int) {
        self.height = height
        self.width = width
    }
    
    func updateK(newK: [Double]) {
        self.K = newK
    }
    
    let R_identity: [Double] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0] // 9 elements
    let P_identity: [Double] = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0] // 12 elements
    let D_empty: [Double] = [] // Empty distortion coefficients
    
    override func constructPayload(frameTime: TimeInterval){
        let rosTime = self.convertTimestampToROS(timestamp: frameTime)
        let payload: [String: Any] = [
            "op": self.op,
            "topic": self.topic,
            "type": self.topicType,
            "msg": [
                "header": ["stamp": rosTime, "frame_id": "camera_depth_frame"],
                "height": self.height,
                "width": self.width,
                "distortion_model": self.distortion_model,
                "k": self.K,
                "r": self.R_identity, // Required fixed array
                "p": self.P_identity, // Required fixed array (4x3 matrix, 12 elements)
                "d": self.D_empty     // Required array
            ]
        ]
        self.msg = payload
    }
    
}


