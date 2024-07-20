#ifndef IOHANDLER_H
#define IOHANDLER_H

#include "guide.h"

#include <Eigen/Dense>
#include <QImage>
#include <filesystem>
#include <string>

#ifdef __APPLE__
namespace fs = std::__fs::filesystem;
#elif defined __linux__
namespace fs = std::filesystem;
#endif

struct Sequence {
	int begFrame;
	int endFrame;
	int step;
	int keyframeIdx;
    int numDigits;
    int size;
	fs::path keyframePath;
    fs::path outputDir;
    fs::path guideDir;
};

class IOHandler
{
public:
    IOHandler(int begFrame, int endFrame, std::string inputFramesDir,
              std::string keyframesDir, std::string outputDir, std::string binaryLocation = "deps/ebsynth/bin/ebsynth");

    void loadInputData(std::vector<std::shared_ptr<QImage>>& inputFrames,
			           std::vector<std::shared_ptr<QImage>>& keyframes);

	void exportImages(const std::vector<std::shared_ptr<QImage>>& images,
                      const fs::path outputDir);
	void exportImages(const std::vector<std::shared_ptr<QImage>>& images, 
                      const fs::path outputDir,
					  const std::vector<fs::path>& filenames);

	int getInputFrameNum(int frameIdx);
	int getKeyframeNum(int keyframeIdx);
    std::vector<Sequence> getSequences(int keyframeIdx);
	std::vector<int> getInputFrameNums();
	std::vector<int> getKeyframeNums();
    fs::path exportGuide(Sequence& s, int frameNum, Guide& g);
    static fs::path getOutputPath(Sequence& s, int frameNum);
    //static fs::path getErrorPath(Sequence &s, int frameNum);
    //fs::path getFlowPath(int frameNum);

    fs::path getInputPath(Sequence &s, int frameNum);
    fs::path getBinaryLocation() const;

private:
    void collectImageFilepaths();
	int calcNumDigits(int num);

    int _begFrame;
    int _endFrame;
    fs::path _inputFramesDir;
    fs::path _keyframesDir;
    fs::path _outputDir;
    fs::path _binaryLocation;
    std::vector<fs::path> _inputFramePaths;
    std::vector<fs::path> _keyframePaths;
	std::vector<int> _inputFrameNums;
    std::vector<int> _keyframeNums;
    Sequence makeSequence(int begFrame, int endFrame, int step, int keyframeIdx);
};

#endif // IOHANDLER_H
