// [N6] Load an ORB-SLAM3 atlas (.osa) and dump it as a coloured PLY point cloud.
// The atlas path comes from System.LoadAtlasFromFile in the yaml (run from the dir
// holding the .osa, since ORB-SLAM3 reads "./<name>.osa"). No frames are processed.
//
//   dump_map <vocab> <yaml-with-LoadAtlasFromFile> <out.ply>
#include <iostream>
#include <System.h>

int main(int argc, char **argv)
{
    if (argc != 4) {
        std::cerr << "usage: dump_map <vocab> <yaml> <out.ply>" << std::endl;
        return 1;
    }
    // viewer off; constructing with a LoadAtlasFromFile yaml deserializes the atlas.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, false);
    SLAM.SaveMapPLY(argv[3]);
    SLAM.Shutdown();
    return 0;
}
