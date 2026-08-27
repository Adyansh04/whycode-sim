// Decodes a WhyCode bit pattern using the project's own CNecklace implementation.
//
// Links the real class rather than reimplementing the necklace maths, so the answer is
// the id the detector will report, not a second interpretation of the encoding.
//
// Usage: decode_main <bits> <samples> <hamming> <code> [<code> ...]
//   where each <code> is a string of 2*bits characters of '0'/'1'.

#include <cstdio>
#include <cstring>
#include <string>

#include "whycode/core/CNecklace.hpp"

int main(int argc, char** argv)
{
    if (argc < 5)
    {
        std::fprintf(stderr, "usage: %s <bits> <samples> <hamming> <code>...\n", argv[0]);
        return 1;
    }

    const int bits    = std::atoi(argv[1]);
    const int samples = std::atoi(argv[2]);
    const int hamming = std::atoi(argv[3]);

    whycon::CNecklace decoder(bits, samples, hamming);

    for (int arg = 4; arg < argc; arg++)
    {
        std::string code(argv[arg]);
        if ((int)code.size() != bits * 2)
        {
            std::fprintf(stderr, "code %s is %zu chars, expected %d\n",
                         argv[arg], code.size(), bits * 2);
            return 1;
        }

        char real[64] = {0};
        whycon::SDecoded out = decoder.decode(code.data(), real, 0, 1.0f, 0.0f);
        std::printf("%s -> id %d (realCode %s, edgeIndex %d)\n",
                    argv[arg], out.id, real, out.edgeIndex);
    }
    return 0;
}
