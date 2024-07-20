#ifndef STYLIZER_H
#define STYLIZER_H

#include "QImage"
//#include "guide.h"
#include "gedge.h"
#include "gpos.h"
#include "gmask.h"
#include "gtemp.h"
#include "iohandler.h"
#include <memory>

class Stylizer {
public:
    Stylizer(std::vector<std::shared_ptr<QImage>> inputFrames, std::vector<std::shared_ptr<QImage>> keyFrames, IOHandler &io);
    virtual ~Stylizer();
    void generateGuides(std::shared_ptr<QImage> key, Sequence& s);
    void run();
    
private:
    std::vector<std::shared_ptr<QImage>> m_frames;
    std::vector<std::shared_ptr<QImage>> m_keys;
    std::vector<std::shared_ptr<QImage>> m_output;
    std::vector<Sequence> m_seqs;

    IOHandler m_io;
};


#endif // STYLIZER_H
