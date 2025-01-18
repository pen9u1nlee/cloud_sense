#include <sqlite3.h>
#include <string>
#include <vector>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <iostream>

class FrameDatabase {
public:
    FrameDatabase(const std::string& dbPath) : dbPath(dbPath) {
        connect();
        createTable();
    }

    ~FrameDatabase() {
        close();
    }

    void insertFrame(int frameNumber, const std::string& frameContent) {
        std::string timestamp = getCurrentTimestamp();
        std::string sql = "INSERT INTO frame_data (frame_number, timestamp, frame_content) VALUES (?, ?, ?);";
        sqlite3_stmt* stmt;
        sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr);
        sqlite3_bind_int(stmt, 1, frameNumber);
        sqlite3_bind_text(stmt, 2, timestamp.c_str(), -1, SQLITE_STATIC);
        sqlite3_bind_blob(stmt, 3, frameContent.data(), frameContent.size(), SQLITE_STATIC);

        if (sqlite3_step(stmt) != SQLITE_DONE) {
            std::cerr << "Error inserting frame: " << sqlite3_errmsg(db) << std::endl;
        }
        sqlite3_finalize(stmt);
    }

    std::vector<std::tuple<int, int, std::string, std::string>> fetchAllFrames() {
        std::vector<std::tuple<int, int, std::string, std::string>> frames;
        std::string sql = "SELECT * FROM frame_data;";
        sqlite3_stmt* stmt;
        sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr);

        while (sqlite3_step(stmt) == SQLITE_ROW) {
            int id = sqlite3_column_int(stmt, 0);
            int frameNumber = sqlite3_column_int(stmt, 1);
            std::string timestamp = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
            std::string frameContent(reinterpret_cast<const char*>(sqlite3_column_blob(stmt, 3)), 
                                      sqlite3_column_bytes(stmt, 3));
            frames.emplace_back(id, frameNumber, timestamp, frameContent);
        }
        sqlite3_finalize(stmt);
        return frames;
    }

    void deleteFrame(int frameNumber) {
        std::string sql = "DELETE FROM frame_data WHERE frame_number = ?;";
        sqlite3_stmt* stmt;
        sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr);
        sqlite3_bind_int(stmt, 1, frameNumber);

        if (sqlite3_step(stmt) != SQLITE_DONE) {
            std::cerr << "Error deleting frame: " << sqlite3_errmsg(db) << std::endl;
        }
        sqlite3_finalize(stmt);
    }

    std::string getDbLocation() const {
        return dbPath;
    }

private:
    sqlite3* db;
    std::string dbPath;

    void connect() {
        if (sqlite3_open(dbPath.c_str(), &db) != SQLITE_OK) {
            std::cerr << "Error opening database: " << sqlite3_errmsg(db) << std::endl;
        }
    }

    void createTable() {
        std::string sql = R"(
            CREATE TABLE IF NOT EXISTS frame_data (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                frame_number INTEGER NOT NULL,
                timestamp TEXT NOT NULL,
                frame_content BLOB NOT NULL
            );
        )";
        if (sqlite3_exec(db, sql.c_str(), nullptr, nullptr, nullptr) != SQLITE_OK) {
            std::cerr << "Error creating table: " << sqlite3_errmsg(db) << std::endl;
        }
    }

    std::string getCurrentTimestamp() {
        std::time_t now = std::time(nullptr);
        std::tm* tm = std::localtime(&now);
        std::ostringstream oss;
        oss << std::put_time(tm, "%Y-%m-%dT%H:%M:%S");
        return oss.str();
    }

    void close() {
        sqlite3_close(db);
    }
};

int main() {
    // 数据库路径
    std::string dbPath = "frame_data.db";

    // 初始化 FrameDatabase
    FrameDatabase frameDb(dbPath);

    // 查询数据库存储位置
    std::cout << "Database storage location: " << frameDb.getDbLocation() << std::endl;

    // 插入帧数据
    frameDb.insertFrame(1, "frame_content_1");
    frameDb.insertFrame(2, "frame_content_2");

    // 查询所有帧数据
    std::cout << "All Frames:" << std::endl;
    auto frames = frameDb.fetchAllFrames();
    for (const auto& frame : frames) {
        std::cout << "ID: " << std::get<0>(frame)
                  << ", Frame Number: " << std::get<1>(frame)
                  << ", Timestamp: " << std::get<2>(frame)
                  << ", Frame Content: " << std::get<3>(frame) << std::endl;
    }

    // 删除帧数据
    frameDb.deleteFrame(1);

    // 再次查询
    std::cout << "\nFrames after deletion:" << std::endl;
    frames = frameDb.fetchAllFrames();
    for (const auto& frame : frames) {
        std::cout << "ID: " << std::get<0>(frame)
                  << ", Frame Number: " << std::get<1>(frame)
                  << ", Timestamp: " << std::get<2>(frame)
                  << ", Frame Content: " << std::get<3>(frame) << std::endl;
    }

    return 0;
}