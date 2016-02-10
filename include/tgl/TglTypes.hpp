#ifndef TGL_TGLTYPES_H
#define TGL_TGLTYPES_H

namespace tgl
{

enum TglMessage {
    TGL_ERROR =         -1,     // can use this for conditional statement
    TGL_WARNING,        // 0
    TGL_OK,             // 1
    TGL_START,          // 2
    TGL_RUNNING,        // 3
    TGL_FINISHED        // 4
};

} // End of namespace tgl
#endif //TGL_TGLTYPES_H
