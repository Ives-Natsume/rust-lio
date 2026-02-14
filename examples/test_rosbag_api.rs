// Test to discover MessageRecord structure
use rosbag::{RosBag, ChunkRecord};

fn main() {
    let bag = RosBag::new("reference/dataset/livox_loop_2025-02-06-18-24-40.bag").unwrap();
    
    for record_result in bag.chunk_records() {
        if let Ok(ChunkRecord::Chunk(chunk)) = record_result {
            for msg_result in chunk.messages() {
                if let Ok(msg) = msg_result {
                    // Print what fields are available
                    println!("MessageRecord type: {:?}", std::any::type_name_of_val(&msg));
                    // Try to access fields we know exist from docs
                    break;
                }
            }
            break;
        }
    }
}
