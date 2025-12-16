//
//  DataExtractor.swift
//  iOSDepthToROS2
//
//  Created by Ayan Syed on 12/15/25.
//

import Foundation
import ARKit
import CoreVideo
import simd

struct DataExtractor {
    /// Converts an ARDepthData CVPixelBuffer into a raw Swift Data object.
    static func extractRawDepthData(from depthData: ARDepthData) -> (data: Data, width: Int, height: Int) {
        
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
    
    // Note: This function is written to extract the raw, bi-planar NV12 data.
    // The ROS 2 client will need to know the specific encoding (e.g., "nv12")
    // and the stride/step for both planes.
    static func extractRawImageData(from pixelBuffer: CVPixelBuffer) -> (data: Data, width: Int, height: Int) {
        
        // Check pixel format to ensure it's the expected ARKit format (NV12)
        guard CVPixelBufferGetPixelFormatType(pixelBuffer) == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange else {
            print("Warning: Captured image is not the expected NV12 format.")
            return (Data(), CVPixelBufferGetWidth(pixelBuffer), CVPixelBufferGetHeight(pixelBuffer))
        }

        let width = CVPixelBufferGetWidth(pixelBuffer)
        let height = CVPixelBufferGetHeight(pixelBuffer)

        // 1. Lock the base address to access the raw memory
        CVPixelBufferLockBaseAddress(pixelBuffer, .readOnly)
        
        // --- Plane 0: Y (Luminance) ---
        let y_plane_base = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0)
        let y_plane_size = CVPixelBufferGetDataSize(pixelBuffer) // This gives total buffer size, not just one plane

        // Calculate the size of the Y plane (width * height * 1 byte/pixel)
        let y_plane_row_bytes = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0)
        let y_plane_byte_count = y_plane_row_bytes * height
        
        guard let y_baseAddress = y_plane_base else {
            CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly)
            return (Data(), width, height)
        }

        // --- Plane 1: UV (Chroma) ---
        let uv_plane_base = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1)
        
        // UV plane is subsampled (half height and half width)
        let uv_plane_height = CVPixelBufferGetHeightOfPlane(pixelBuffer, 1)
        let uv_plane_row_bytes = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 1)
        let uv_plane_byte_count = uv_plane_row_bytes * uv_plane_height

        guard let uv_baseAddress = uv_plane_base else {
            CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly)
            return (Data(), width, height)
        }

        // 2. Combine the data from both planes into a single Data object
        var combinedData = Data(bytes: y_baseAddress, count: y_plane_byte_count)
        combinedData.append(Data(bytes: uv_baseAddress, count: uv_plane_byte_count))
        
        // 3. Unlock the base address
        CVPixelBufferUnlockBaseAddress(pixelBuffer, .readOnly)

        // ROS Encoding: "nv12"
        return (combinedData, width, height)
    }
    
    /// Converts the ARDepthData confidence map CVPixelBuffer into a raw Swift Data object.
    static func extractRawConfidenceData(from depthData: ARDepthData) -> (data: Data, width: Int, height: Int) {
        
        // 1. SAFELY UNWRAP the confidenceMap
        guard let confidencePixelBuffer = depthData.confidenceMap else {
            return (Data(), 0, 0) // Return nil data and zero dimensions if the map is unavailable
        }
        
        // Now confidencePixelBuffer is guaranteed to be a non-optional CVPixelBuffer
        
        let width = CVPixelBufferGetWidth(confidencePixelBuffer)
        let height = CVPixelBufferGetHeight(confidencePixelBuffer)
        
        // 2. Lock the base address
        CVPixelBufferLockBaseAddress(confidencePixelBuffer, .readOnly)
        
        // Get a pointer to the start of the data, and SAFELY UNWRAP it
        guard let baseAddress = CVPixelBufferGetBaseAddress(confidencePixelBuffer) else {
            CVPixelBufferUnlockBaseAddress(confidencePixelBuffer, .readOnly)
            return (Data(), width, height)
        }
        
        // Calculate the total size of the data in bytes
        let bytesPerRow = CVPixelBufferGetBytesPerRow(confidencePixelBuffer)
        let totalBytes = bytesPerRow * height
        
        // 3. Create the Swift Data object by copying the bytes
        let data = Data(bytes: baseAddress, count: totalBytes)
        
        // 4. Unlock the base address
        CVPixelBufferUnlockBaseAddress(confidencePixelBuffer, .readOnly)
        
        return (data, width, height)
    }
    
    
    static func getPoseTransform(frame: ARFrame) -> simd_float4x4 {
        let arTransform = frame.camera.transform
        return arTransform
    }
    
    static func getCameraInfo(frame: ARFrame) -> (Int, Int, [Double]){
        let camera = frame.camera
        let resolution = camera.imageResolution
        let intrinsics = camera.intrinsics // This is a simd_float3x3 matrix
        
        // 1. Flatten the intrinsics matrix (focal length fx, fy, principal point cx, cy)
        // ROS K matrix is a 3x3 row-major array.
        // [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        
        let K: [Double] = [
            // Row 0: fx, 0, cx
            Double(intrinsics[0][0]), Double(intrinsics[0][1]), Double(intrinsics[0][2]),
            // Row 1: 0, fy, cy
            Double(intrinsics[1][0]), Double(intrinsics[1][1]), Double(intrinsics[1][2]),
            // Row 2: 0, 0, 1
            Double(intrinsics[2][0]), Double(intrinsics[2][1]), Double(intrinsics[2][2])
        ]
        
        return (Int(resolution.width), Int(resolution.height), K)
    }
    
    
//Tried Implementing IMU, but i think its only for visionOS, not completely necessary so will skip for now
//    static func getIMUData(frame: ARFrame) -> (acc: simd_double3, rot: simd_double3) {
//        guard let motion = frame.camera.image. else { return (simd_double3(), simd_double3())}
//        let rotationRate = motion.rotationRate
//        let userAcceleration = motion.userAcceleration
//        return (userAcceleration, rotationRate)
//    }
}
