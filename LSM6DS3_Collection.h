struct pCollection_args {
	uint8_t BLOCK_READY;
	float x;
	float y;
	float z;
};

struct pCollection_args *gArgs;

void *runCollection(void *bp);

