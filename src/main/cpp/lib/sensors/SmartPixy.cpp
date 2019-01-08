#include "lib/sensors/SmartPixy.h"
#include "unistd.h"
#include "stdlib.h"

namespace frc973 {

static constexpr bool PIXY_SIM_NOISE = false;
int num_cs_errors = 0; /**< The number of CS errors. */

#define RUN_LOG_ERR(x) \
    { x; }
//{int ret=x;if((ret)) { fprintf(stderr, "Warning: " #x " returned %d\n",
// ret);}}

Pixy::Pixy()
        : i2c(new I2C(I2C::Port::kOnboard, PIXY_I2C_DEFAULT_ADDR))
        , blockType(BlockType::NORMAL_BLOCK)
        , skipStart(false)
        , blockCount(0) {
}

Pixy::~Pixy() {
    delete i2c;

    i2c = nullptr;
}

void Pixy::Block::print() {
    if (signature > PIXY_MAX_SIGNATURE)  // color code! (CC)
    {
        printf(
            "CC block! sig: 0x%x (%d decimal) x: %d y: %d width: %d height: %d "
            "angle %d\n",
            signature, signature, x, y, width, height, angle);
    }
    else  // regular block.  Note, angle is always zero, so no need to print
        printf("sig: 0x%x x: %d y: %d width: %d height: %d\n", signature, x, y,
               width, height);  // prints out data to console instead of
                                // smartDashboard -> check on the side of the
                                // driver station, check +print and click view
                                // console
}

bool Pixy::getStart() {
    uint16_t lastw;

    lastw = 0xffff;

    int i = 0;
    while (true) {
        uint16_t w = getWord();
        if (w == 0 && lastw == 0) {
            return false;
        }
        else if (w == PIXY_START_WORD && lastw == PIXY_START_WORD) {
            blockType = NORMAL_BLOCK;
            return true;
        }
        else if (w == PIXY_START_WORD_CC && lastw == PIXY_START_WORD) {
            blockType = CC_BLOCK;
            return true;
        }
        else if (w == PIXY_START_WORDX) {
            // when byte received was 0x55aa instead of otherway around, the
            // code syncs the byte
            printf("Pixy: reorder");
            getByte();  // resync
        }
        lastw = w;
        if (i++ == 100) {
            usleep(5 * 1000);
            printf("done 100 iterations waiting for the start\n");
            i = 0;
        }
    }
}

uint16_t Pixy::getWord() {
    unsigned char buffer[2] = {0, 0};
    uint16_t ret;

    RUN_LOG_ERR(i2c->ReadOnly(1, buffer + 0));
    RUN_LOG_ERR(i2c->ReadOnly(1, buffer + 1));
    ret = (buffer[1] << 8) | buffer[0];

    if (PIXY_SIM_NOISE && rand() % 50 == 1) {
        ret++;
    }
    return ret;
}

uint8_t Pixy::getByte() {
    unsigned char buffer[1] = {0};

    RUN_LOG_ERR(i2c->ReadOnly(1, buffer));
    if (PIXY_SIM_NOISE && rand() % 50 == 1) {
        buffer[0]++;
    }
    return buffer[0];
}

uint16_t Pixy::getBlocks(uint16_t maxBlocks) {
    // printf("Starting frame\n");
    blocks[0] = {0};  // resets the array - clears out data from previous
                      // reading
    uint8_t i;
    uint16_t w, checksum, sum;
    Block *block;

    if (!skipStart)  // when computer has not seen 0xaa55 (starting frame)
    {
        if (getStart() == false)
            return 0;
    }
    else
        skipStart = false;

    for (blockCount = 0;
         blockCount < maxBlocks && blockCount < PIXY_MAXIMUM_ARRAYSIZE;) {
        checksum = getWord();
        if (checksum == PIXY_START_WORD)  // we've reached the beginning of the
                                          // next frame - checking for 0xaa55
        {
            skipStart = true;  // starts this function
            blockType = NORMAL_BLOCK;
            return blockCount;
        }
        else if (checksum == PIXY_START_WORD_CC)  // we've reacehd the beginning
                                                  // of the next frame -
                                                  // checking for 0xaa56
        {
            skipStart = true;
            blockType = CC_BLOCK;
            return blockCount;
        }
        else if (checksum == 0) {
            return blockCount;
        }

        block = blocks + blockCount;

        for (i = 0, sum = 0; i < sizeof(Block) / sizeof(uint16_t); i++) {
            if (blockType == NORMAL_BLOCK && i >= 5) {
                // skip --if not an CC block, no need to consider angle
                block->angle = 0;
                break;
            }
            w = getWord();
            sum += w;
            *((uint16_t *)block + i) = w;
        }

        if (checksum == sum) {
            blockCount++;
            block->print();
        }
        else {
            printf("Pixy: cs error %d\n", ++num_cs_errors);
        }

        w = getWord();  // when this is start of the frame
        if (w == PIXY_START_WORD) {
            blockType = NORMAL_BLOCK;
        }
        else if (w == PIXY_START_WORD_CC) {
            blockType = CC_BLOCK;
        }
        else {
            return blockCount;
        }
    }
    return blockCount;
}
}
