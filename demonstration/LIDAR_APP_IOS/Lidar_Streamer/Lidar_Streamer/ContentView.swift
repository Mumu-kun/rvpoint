import SwiftUI
import ARKit
import SceneKit

struct ContentView: View {
    @StateObject private var streamer = LiDARStreamer()

    var body: some View {
        ZStack {
            ARViewContainer(streamer: streamer).ignoresSafeArea()
            VStack {
                Spacer()
                VStack(spacing: 10) {
                    TextField("Mac IP address", text: $streamer.host)
                        .textFieldStyle(.roundedBorder)
                        .keyboardType(.numbersAndPunctuation)
                        .padding(.horizontal)
                    Button(streamer.streaming ? "Stop streaming" : "Start streaming") {
                        streamer.toggleStreaming()
                    }
                    .buttonStyle(.borderedProminent)
                    .tint(streamer.streaming ? .red : .green)
                    Text(streamer.status)
                        .font(.caption).foregroundColor(.white)
                        .padding(8).background(Color.black.opacity(0.55)).cornerRadius(8)
                    Text("Frames sent: \(streamer.framesSent)")
                        .font(.caption2).foregroundColor(.white)
                        .padding(4).background(Color.black.opacity(0.55)).cornerRadius(6)
                }
                .padding(.bottom, 30)
            }
        }
    }
}

struct ARViewContainer: UIViewRepresentable {
    let streamer: LiDARStreamer

    func makeUIView(context: Context) -> ARSCNView {
        let view = ARSCNView(frame: .zero)
        streamer.attach(to: view)
        return view
    }
    func updateUIView(_ uiView: ARSCNView, context: Context) {}
}
