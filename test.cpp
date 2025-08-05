// https://chatgpt.com/c/688aeed1-13b8-8330-bd37-376dbded2c91

/*

We can use std::future and std::promise to replace the loading_ set and condition variable logic. 
This is a modern and elegant solution to ensure:

Only one thread loads a missing tile.
Other threads wait on the same future to get the result.
No need for condition_variable, loading_ set, or manual signaling.



  std::future is very useful, especially to avoid redundant slow tile loads.
  Basically std::future makes sure only one thread is loading, and the other threads only needs to wait. 

  1. The std::future in C++ is a standard library facility that is tightly connected to multithreaded programming. 
  2. It provides a mechanism to access the result of an asynchronous operation, typically executed in a separate thread. 
  
  Use std::future<T> to let readers wait on results only when the tile is not ready and is being written by a background thread.


  Here's the relationship and key concepts:
    Relationship to Multithreading
    1. Futures Enable Communication Between Threads
       In multithreaded programming, you often launch a task in a new thread and want to get the result later.
       std::future provides a safe way to get that result from the thread after it finishes.
    2. Used with std::async
       One common way to use std::future is with std::async, which starts a new thread (or uses a thread pool internally) and returns a std::future that holds the result:
       


       */

#include <iostream>
#include <unordered_map>
#include <future>
#include <thread>
#include <chrono>
#include <mutex>

std::mutex cache_mutex;

using TileID = int;
using TileData = std::string;

std::unordered_map<TileID, std::shared_future<TileData>> tile_cache;

TileData loadTileFromDisk(TileID id) {
    std::this_thread::sleep_for(std::chrono::seconds(5)); // simulate expensive I/O
    return "Tile_" + std::to_string(id);
}

std::shared_future<TileData> getTile(TileID id) {
    std::lock_guard<std::mutex> lock(cache_mutex);

    auto it = tile_cache.find(id);
    if (it != tile_cache.end()) {
        // Cache hit
        return it->second;
    }

    // Cache miss – create a promise and future pair
    // You can also manually create a producer-consumer model using std::promise and std::future. The promise is set in one thread, and the future retrieves it in another
    std::promise<TileData> prom;
    std::shared_future<TileData> fut = prom.get_future().share(); // .share() allows multiple readers

    tile_cache[id] = fut; // insert future immediately to allow other readers to wait

    // Start background thread to load the tile and fulfill the promise
    // Use detatch to run in the background, while join is runnin in the foreground
    std::thread([id, p = std::move(prom)]() mutable {
        TileData data = loadTileFromDisk(id);
        p.set_value(data); // set the data
    }).detach();

    return fut;
}

void readerThread(TileID id) {
    auto fut = getTile(id);
    std::cout << "Reader thread waiting for tile " << id << "\n";
    TileData data = fut.get(); // wait here if needed


    std::cout << "Reader got tile " << id << ": " << data << "\n";
}

int main() {
    std::thread t1(readerThread, 1);
    std::thread t2(readerThread, 1); 
    std::thread t3(readerThread, 2);
    std::thread t4(readerThread, 1);
    
    

    t1.join();
    t2.join();
    t3.join();
    t4.join();
    

    return 1;
}
