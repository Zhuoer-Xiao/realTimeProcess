// cloud_worker.h
#ifndef CLOUD_WORKER_H
#define CLOUD_WORKER_H

#include <QObject>
#include <QFileSystemWatcher>
#include <QFileInfo>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template<typename PointT>
struct CloudWithMetadata {
    using Ptr = std::shared_ptr<CloudWithMetadata<PointT>>;
    typename pcl::PointCloud<PointT>::Ptr cloud;
    QString filePath;
    qint64 timestamp;
};

/**
 * @class CloudWorker
 * @brief 支持多格式的点云文件监控与加载工作线程
 *
 * 功能特性：
 * - 实时监控文件夹变化
 * - 自动识别 PCD/PLY 格式
 * - 带时间戳的点云数据封装
 * - 智能文件去重处理
 */
template<typename PointT>
class CloudWorker : public QObject {
    Q_OBJECT
public:
    explicit CloudWorker(QObject* parent = nullptr)
        : QObject(parent), m_minFileInterval(500) // 最小文件处理间隔500ms
    {
        connect(&m_watcher, &QFileSystemWatcher::fileChanged,
            this, &CloudWorker::handleFileChange);
    }

public slots:
    /// 设置监控目录并自动加载已有文件
    void setWatchDirectory(const QString& dir) {
        m_watcher.addPath(dir);
        processExistingFiles(dir);
    }

    /// 设置最小文件处理间隔（防抖动）
    void setMinFileInterval(int ms) { m_minFileInterval = ms; }

signals:
    void cloudReady(typename CloudWithMetadata<PointT>::Ptr data);
    void errorOccurred(const QString& message);

private slots:
    void handleFileChange(const QString& path) {
        QFileInfo fi(path);
        QString ext = fi.suffix().toLower();

        // 格式过滤
        if (ext != "pcd" && ext != "ply") return;

        // 去重检查：同一文件500ms内不重复处理
        qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
        if (m_fileTimestamps.contains(path) &&
            (currentTime - m_fileTimestamps[path]) < m_minFileInterval) {
            return;
        }
        m_fileTimestamps[path] = currentTime;

        // 异步处理防止阻塞
        QTimer::singleShot(0, this, [this, path, ext]() {
            processCloudFile(path, ext);
            });
    }

private:
    /// 处理单个点云文件的核心方法
    void processCloudFile(const QString& path, const QString& format) {
        typename CloudWithMetadata<PointT>::Ptr data(new CloudWithMetadata<PointT>);
        data->filePath = path;
        data->timestamp = QDateTime::currentMSecsSinceEpoch();
        data->cloud.reset(new pcl::PointCloud<PointT>);

        try {
            // 分支处理不同格式
            if (format == "pcd") {
                if (pcl::io::loadPCDFile<PointT>(path.toStdString(), *data->cloud) == -1) {
                    throw std::runtime_error("Invalid PCD file structure");
                }
            }
            else if (format == "ply") {
                pcl::PLYReader reader;
                if (reader.read(path.toStdString(), *data->cloud) == -1) {
                    throw std::runtime_error("Invalid PLY file structure");
                }
            }

            // 统一后处理
            postProcessCloud(data->cloud);

            emit cloudReady(data);
        }
        catch (const std::exception& e) {
            emit errorOccurred(tr("文件加载失败: %1\n原因: %2").arg(path).arg(e.what()));
        }
    }

    /// 点云后处理（坐标系统一等）
    void postProcessCloud(typename pcl::PointCloud<PointT>::Ptr cloud) {
        // 自动转换到Z-up坐标系（如果PLY文件使用Y-up）
        if constexpr (std::is_same_v<PointT, pcl::PointXYZ>) {
            for (auto& point : *cloud) {
                float y = point.y;
                point.y = point.z;
                point.z = y;
            }
        }
    }

    /// 初始化加载已有文件
    void processExistingFiles(const QString& dir) {
        QDir directory(dir);
        QStringList filters = { "*.pcd", "*.ply" };
        auto files = directory.entryInfoList(filters, QDir::Files, QDir::Time);

        for (const auto& fileInfo : files) {
            m_watcher.addPath(fileInfo.absoluteFilePath());
            processCloudFile(fileInfo.absoluteFilePath(),
                fileInfo.suffix().toLower());
        }
    }

    QFileSystemWatcher m_watcher;
    QMap<QString, qint64> m_fileTimestamps; // 文件最后处理时间记录
    int m_minFileInterval; // 文件处理最小间隔（防抖动）
};

#endif // CLOUD_WORKER_H