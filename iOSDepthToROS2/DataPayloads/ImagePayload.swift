//
//  ImagePayload.swift
//  iOSDepthToROS2
//
//  Created by Ayan Syed on 12/14/25.
//

import Foundation

class ImagePayload: Payload{
    let isBigEndian: Int = 0
    let topicType: String = "Image/"
    let msgType: String = "Image"
    
    var encoding: String = "32FC1"
    var height: Int = 0
    var width: Int = 0
    var stepMultiplier = 4 // 4 bytes per Float32
    
    
    init(topicName: String){
        super.init(topicField: (self.topicType + topicName), msgType: self.msgType)
        print("Created Image Topic Class: " + self.topic + " with Type: " + self.type)
    }
    
    init(topicName: String, encoding: String){
        super.init(topicField: (self.topicType + topicName), msgType: self.msgType)
        self.encoding = encoding
        print("Created Image Topic Class: " + self.topic + " with Type: " + self.type)
    }
    
    
    func updateData(data: Data, height: Int, width: Int) {
        super.updateData(info: data)
        self.height = height
        self.width = width
    }
    
    override func constructPayload(){
        let timestampROS = self.getCurrentTimestamp()
        let payload: [String: Any] = [
            "op": self.op,
            "topic": self.topic,
            "type": self.topicType,
            "msg": [
                "header": ["stamp": timestampROS, "frame_id": "camera_depth_frame"],
                "height": self.height,
                "width": self.width,
                "encoding": self.encoding,
                "is_bigendian": self.isBigEndian,
                "step": self.width * self.stepMultiplier,
                "data": self.convertBase64() //Converts class Payload Data to Base 64
            ]
        ]
        self.msg = payload
    }
    
}


