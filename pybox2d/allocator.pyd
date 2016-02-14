cdef extern from "b2BlockAllocator.h":
    const int32 b2_chunkSize
    const int32 b2_maxBlockSize
    const int32 b2_blockSizes
    const int32 b2_chunkArrayIncrement

    # struct b2Block;
    # struct b2Chunk;
    
    cdef cppclass b2BlockAllocator:
        b2BlockAllocator();
        # ~b2BlockAllocator();
    
        void* Allocate(int32 size);
        void Free(void* p, int32 size);
        void Clear();
