#include "stylizer.h"
#include "opencvutils.h"
#include "src/gpos.h"
#include "optical-flow/simpleflow.h"

using namespace std;

Stylizer::Stylizer(std::vector<shared_ptr<QImage>> inputFrames, std::vector<shared_ptr<QImage>> keyFrames, IOHandler &io) :
	m_frames(inputFrames), m_keys(keyFrames), m_io(io)
{
}

Stylizer::~Stylizer(){
	m_frames.clear();
	m_keys.clear();
}

void Stylizer::run(){
    // Load in all Sequences and generate them
    std::vector<Sequence> seqs;
    for (uint i = 0; i < m_keys.size(); i++){
        seqs = m_io.getSequences(i);
        m_seqs.insert(m_seqs.end(), seqs.begin(), seqs.end());
    }
    Sequence cur;
    for (uint i = 0; i < m_seqs.size(); i++) {
        cur = m_seqs.at(i);
        generateGuides(m_keys.at(cur.keyframeIdx), cur);
    }
}

void Stylizer::generateGuides(shared_ptr<QImage> keyframe, Sequence& s) {
	std::shared_ptr<QImage> key(new QImage(*keyframe));
    std::shared_ptr<QImage> mask(new QImage(*m_frames.at(s.begFrame)));
    std::shared_ptr<QImage> frame1(new QImage(*m_frames.at(s.begFrame)));

	// get initial GEdge guide
	GEdge edge(frame1);
    fs::path edge_initial = fs::absolute(m_io.exportGuide(s, s.begFrame, edge));

	// filler mask for advection
	mask->fill(Qt::white);
    GPos gpos_start = GPos(mask);
    fs::path pos_initial = fs::absolute(m_io.exportGuide(s, s.begFrame, gpos_start));
	GPos gpos_cur = gpos_start;
	Mat i1, i2;

	// use keyframe as initial previously stylized frame
    fs::path temp_initial = fs::absolute(s.keyframePath);
    std::shared_ptr<QImage> prevStylizedFrame(new QImage(*keyframe));

    // export keyframe so we have it
    m_io.exportImages(std::vector<std::shared_ptr<QImage>>({prevStylizedFrame}), s.outputDir, std::vector<fs::path>({s.keyframePath.filename()}));

	GTemp gtemp;

    // initial frame of video
    fs::path color_initial = fs::absolute(m_io.getInputPath(s, s.begFrame));

	// going either forwards or backwards depending on keyframe
    Ptr<DenseOpticalFlow> deepflow = cv::optflow::createOptFlow_DeepFlow();
    for (int i = s.begFrame+s.step; i != s.endFrame+s.step; i+=s.step){
        std::shared_ptr<QImage> cur_frame(new QImage(*m_frames.at(i)));

        edge.updateFrame(cur_frame);
        fs::path edge_cur = fs::absolute(m_io.exportGuide(s, i, edge));

        i1 = qimage_to_mat_ref((*m_frames.at(i-s.step)));
		i2 = qimage_to_mat_ref((*m_frames.at(i)));

                cvtColor(i1, i1, COLOR_BGRA2GRAY);
                cvtColor(i2, i2, COLOR_BGRA2GRAY);

                Mat2f out;
                deepflow->calc(i1, i2, out);//calculateFlow(i1, i2, false, false);
                cv::patchNaNs(out, 0);

		// if running through whole pipeline, store advection field
        if (s.step > 0) {
            serializeMatbin(out, m_io.getFlowPath(i));
        }

		// get GPos and GTemp guides
		gpos_cur.advect(mask, out);
        fs::path pos_cur = fs::absolute(m_io.exportGuide(s, i, gpos_cur));

		gtemp.updateGuide(prevStylizedFrame, out, mask);
        fs::path temp_cur = fs::absolute(m_io.exportGuide(s, i, gtemp));

        // get current frame of video
        fs::path color_cur = fs::absolute(m_io.getInputPath(s, i));

		// build command to call ebsynth
        std::string command = m_io.getBinaryLocation().string() + " -style " + fs::absolute(s.keyframePath).string();
        command += " -guide " + edge_initial.string() +  " " + edge_cur.string() + " -weight 0.5 ";

        //command.append("-guide " + g_mask1 + " " + g_mask2 + " -weight 6 ");

        command+="-guide " + color_initial.string() +  " " + color_cur.string() + " -weight 6 ";

        command+="-guide " + pos_initial.string() + " " + pos_cur.string() + " -weight 2 ";

        command+="-guide " + temp_initial.string() + " " + temp_cur.string() + " -weight 0.5 ";

        command+=("-output " + fs::absolute(m_io.getOutputPath(s, i)).string());

        command+=" -searchvoteiters 12 -patchmatchiters 6";

        const char *c_str = command.c_str();
		// actually calls ebsynth executable
        std::system(c_str);
        QImage prevStylized;
        Mat intermediate = cv::imread(fs::relative(m_io.getOutputPath(s, i)).string());
        cv::cvtColor(intermediate, intermediate, cv::COLOR_BGR2RGB);
        prevStylized = QImage(intermediate.data, intermediate.cols, intermediate.rows, intermediate.step, QImage::Format_RGB888).copy();
        prevStylizedFrame = std::make_shared<QImage>(prevStylized);
	}
}
