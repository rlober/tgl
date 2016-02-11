#ifndef TGL_TGLTYPES_H
#define TGL_TGLTYPES_H

namespace tgl
{

enum TglMessage {
    TGL_ERROR,          // 0 can use this for conditional statement
    TGL_OK,             // 1
    TGL_WARNING,        // 2
    TGL_START,          // 3
    TGL_RUNNING,        // 4
    TGL_FINISHED        // 5
};

} // End of namespace tgl
#endif //TGL_TGLTYPES_H
