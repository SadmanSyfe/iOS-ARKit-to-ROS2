//
//  ImagePayload.swift
//  iOSDepthToROS2
//
//  Created by Ayan Syed on 12/14/25.
//

import Foundation
import simd
import ARKit

class TransformStampedPayload: Payload{
    let isBigEndian: Int = 0
    let topicType: String = "Pose/"
    let msgType: String = "TransformStamped"
    let rotation = simd_float4x4(
            simd_float4( 0, -1,  0, 0),  // ROS X axis = -AR Y axis (simpler common conversion)
            simd_float4( 0,  0, -1, 0),  // ROS Y axis = -AR Z axis
            simd_float4( 1,  0,  0, 0),  // ROS Z axis = AR X axis
            simd_float4( 0,  0,  0, 1)
        )
    
    var x: Double = 0.0
    var y: Double = 0.0
    var z: Double = 0.0
    var quaternion: simd_quatf = simd_quatf()

    
    init(topicName: String){
        super.init(topicField: (self.topicType + topicName), msgType: self.msgType)
        self.type = "geometry_msgs/msg/TransformStamped"
        print("Created Image Topic Class: " + self.topic + " with Type: " + self.type)
    }
    
    
    
    func updateTransform(newTransform: simd_float4x4) {
        let arTransform = newTransform
        let rosTransform = arTransform * rotation
        let translation = rosTransform.columns.3
        self.x = Double(translation.x)
        self.y = Double(translation.y)
        self.z = Double(translation.z)
        let rotationMatrix = simd_float3x3(
            // Column 0 of the 4x4 becomes Column 0 of the 3x3
            simd_float3(rosTransform.columns.0.x, rosTransform.columns.0.y, rosTransform.columns.0.z),
            
            // Column 1 of the 4x4 becomes Column 1 of the 3x3
            simd_float3(rosTransform.columns.1.x, rosTransform.columns.1.y, rosTransform.columns.1.z),
            
            // Column 2 of the 4x4 becomes Column 2 of the 3x3
            simd_float3(rosTransform.columns.2.x, rosTransform.columns.2.y, rosTransform.columns.2.z)
        )
        self.quaternion = simd_quatf(rotationMatrix)
    }
    
    
    override func constructPayload(){
        let timestampROS = self.getCurrentTimestamp()
        let payload: [String: Any] = [
                "op": self.op,
                "topic": self.topic,
                "type": self.topicType,
                "msg": [
                    "header": [
                        "stamp": timestampROS,
                        "frame_id": "odom" // The parent frame (fixed world)
                    ],
                    "child_frame_id": "base_link", // The moving frame (your camera/robot)
                    "transform": [
                        "translation": ["x": self.x, "y": self.y, "z": self.z],
                        "rotation": [
                            "x": Double(self.quaternion.vector.x),
                            "y": Double(self.quaternion.vector.y),
                            "z": Double(self.quaternion.vector.z),
                            "w": Double(self.quaternion.vector.w)
                        ]
                    ]
                ]
            ]
        self.msg = payload
    }
    

    
}


