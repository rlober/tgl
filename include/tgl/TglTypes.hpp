#ifndef TGL_TGLTYPES_H
#define TGL_TGLTYPES_H

namespace tgl
{

enum TglMessage {
    TGL_ERROR = -1,     // can use this for conditional statement
    TGL_WARNING,        // 0
    TGL_START,          // 1
    TGL_RUNNING,        // 2
    TGL_FINISHED        // 3
};

} // End of namespace tgl
#endif //TGL_TGLTYPES_H
