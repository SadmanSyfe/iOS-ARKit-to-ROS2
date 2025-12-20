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
import CoreMotion


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
	
	static func extractDownscaledDepthData(from depthData: ARDepthData, scale: CGFloat = 0.75) -> (data: Data, width: Int, height: Int) {
	    let depthPixelBuffer = depthData.depthMap
	    let ciImage = CIImage(cvPixelBuffer: depthPixelBuffer)
	    
	    // 1. Calculate the new dimensions
	    let originalWidth = CVPixelBufferGetWidth(depthPixelBuffer)
	    let originalHeight = CVPixelBufferGetHeight(depthPixelBuffer)
	    let newWidth = Int(CGFloat(originalWidth) * scale)
	    let newHeight = Int(CGFloat(originalHeight) * scale)
	    
	    // 2. Apply the scaling transform
	    let scaledImage = ciImage.transformed(by: CGAffineTransform(scaleX: scale, y: scale))
	    
	    // 3. Prepare the destination buffer (4 bytes per pixel for Float32)
	    let context = CIContext(options: [.useSoftwareRenderer: false])
	    let bytesPerPixel = MemoryLayout<Float32>.size
	    let rowBytes = newWidth * bytesPerPixel
	    let totalBytes = rowBytes * newHeight
	    
	    var data = Data(count: totalBytes)
	    
	    // 4. Render to the Data object
	    data.withUnsafeMutableBytes { ptr in
		   if let baseAddress = ptr.baseAddress {
			  context.render(scaledImage,
						  toBitmap: baseAddress,
						  rowBytes: rowBytes,
						  bounds: CGRect(x: 0, y: 0, width: newWidth, height: newHeight),
						  format: .Rf, // Critical: R-channel Float (32-bit)
						  colorSpace: nil) // Depth is non-color data
		   }
	    }
	    
	    return (data, newWidth, newHeight)
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
	
	// Persistent state for velocity calculation
	    private static var lastPosition: simd_float3?
	    private static var lastTimestamp: TimeInterval?
	// Initialize Motion Manager once
	    private static let motionManager = CMMotionManager()

		static func startMotionUpdates() {
		   if motionManager.isDeviceMotionAvailable {
			  motionManager.deviceMotionUpdateInterval = 1.0 / 60.0
			  motionManager.startDeviceMotionUpdates()
		   }
	    }

	    static func getTwist(frame: ARFrame) -> (linear: simd_float3, angular: simd_float3) {
		   let currentTime = frame.timestamp
		   let currentPos = simd_float3(frame.camera.transform.columns.3.x,
								  frame.camera.transform.columns.3.y,
								  frame.camera.transform.columns.3.z)
		   
		   var linearVelocity = simd_float3(0, 0, 0)
		   
		   // 1. Calculate Linear Velocity (dx/dt)
		   if let lastPos = lastPosition, let lastTime = lastTimestamp {
			  let dt = Float(currentTime - lastTime)
			  if dt > 0 {
				 let deltaPos = currentPos - lastPos
				 let rawVelocity = deltaPos / dt
				 // Apply your working ROS coordinate shuffle
				 linearVelocity = simd_float3(-rawVelocity.z, -rawVelocity.x, rawVelocity.y)
			  }
		   }
		   
		   lastPosition = currentPos
		   lastTimestamp = currentTime
		   
		   // 2. Get Angular Velocity Synchronously
		   var angularVelocity = simd_float3(0, 0, 0)
		   if let motion = motionManager.deviceMotion {
			  let rate = motion.rotationRate // CMRotationRate (x, y, z) in rad/s
			  
			  // Map to ROS axes based on your working rotation logic
			  angularVelocity = simd_float3(Float(rate.z), Float(rate.x), Float(rate.y))
		   }
		   
		   return (linearVelocity, angularVelocity)
	    }
	
	
	static func extractRGB8ImageData(from pixelBuffer: CVPixelBuffer) -> (data: Data, width: Int, height: Int) {
	    let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
	    let context = CIContext()
	    
	    let width = CVPixelBufferGetWidth(pixelBuffer)
	    let height = CVPixelBufferGetHeight(pixelBuffer)
	    
	    // 1. Create a buffer for the RGB data (3 bytes per pixel: R, G, B)
	    let bytesPerPixel = 3
	    let rgbDataSize = width * height * bytesPerPixel
	    var rgbData = Data(count: rgbDataSize)
	    
	    // 2. Render the CIImage into the RGB buffer
	    // We specify a standard RGB color space
	    rgbData.withUnsafeMutableBytes { ptr in
		   if let baseAddress = ptr.baseAddress {
			  context.render(ciImage,
						  toBitmap: baseAddress,
						  rowBytes: width * bytesPerPixel,
						  bounds: ciImage.extent,
						  format: .RGBA8, // We render as RGBA then strip A, or use a custom filter
						  colorSpace: CGColorSpaceCreateDeviceRGB())
		   }
	    }
	    
	    // Note: CIContext.render for .RGBA8 produces 4 bytes per pixel.
	    // ROS "rgb8" expects exactly 3 bytes. Let's optimize:
	    
	    return (stripAlpha(from: rgbData, width: width, height: height), width, height)
	}

	// Helper to convert 4-byte RGBA to 3-byte RGB for ROS
	private static func stripAlpha(from rgbaData: Data, width: Int, height: Int) -> Data {
	    var rgbData = Data(capacity: width * height * 3)
	    rgbaData.withUnsafeBytes { ptr in
		   let rgbaPtr = ptr.bindMemory(to: UInt8.self)
		   for i in stride(from: 0, to: width * height * 4, by: 4) {
			  rgbData.append(rgbaPtr[i])     // R
			  rgbData.append(rgbaPtr[i + 1]) // G
			  rgbData.append(rgbaPtr[i + 2]) // B
		   }
	    }
	    return rgbData
	}
	
	
	static func extractDownsampledRGB8Data(from pixelBuffer: CVPixelBuffer, scale: CGFloat = 0.1) -> (data: Data, width: Int, height: Int) {
	    let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
	    
	    // 1. Calculate new dimensions
	    let newWidth = Int(CGFloat(CVPixelBufferGetWidth(pixelBuffer)) * scale)
	    let newHeight = Int(CGFloat(CVPixelBufferGetHeight(pixelBuffer)) * scale)
	    
	    // 2. Apply a scaling transform to the CIImage
	    let scaledImage = ciImage.transformed(by: CGAffineTransform(scaleX: scale, y: scale))
	    
	    let context = CIContext(options: [.useSoftwareRenderer: false])
	    let bytesPerPixel = 4 // Rendering to RGBA8 first
	    let rgbaDataSize = newWidth * newHeight * bytesPerPixel
	    var rgbaData = Data(count: rgbaDataSize)
	    
	    rgbaData.withUnsafeMutableBytes { ptr in
		   if let baseAddress = ptr.baseAddress {
			  context.render(scaledImage,
						  toBitmap: baseAddress,
						  rowBytes: newWidth * bytesPerPixel,
						  bounds: scaledImage.extent,
						  format: .RGBA8,
						  colorSpace: CGColorSpaceCreateDeviceRGB())
		   }
	    }
	    
	    // 3. Convert RGBA to RGB (Strip Alpha)
	    var rgbData = Data(capacity: newWidth * newHeight * 3)
	    rgbaData.withUnsafeBytes { ptr in
		   let rgbaPtr = ptr.bindMemory(to: UInt8.self)
		   for i in stride(from: 0, to: newWidth * newHeight * 4, by: 4) {
			  rgbData.append(rgbaPtr[i])     // R
			  rgbData.append(rgbaPtr[i + 1]) // G
			  rgbData.append(rgbaPtr[i + 2]) // B
		   }
	    }
	    
	    return (rgbData, newWidth, newHeight)
	}
	
	
		    
}
    
    
//Tried Implementing IMU, but i think its only for visionOS, not completely necessary so will skip for now
//    static func getIMUData(frame: ARFrame) -> (acc: simd_double3, rot: simd_double3) {
//        guard let motion = frame.camera.image. else { return (simd_double3(), simd_double3())}
//        let rotationRate = motion.rotationRate
//        let userAcceleration = motion.userAcceleration
//        return (userAcceleration, rotationRate)
//    }
