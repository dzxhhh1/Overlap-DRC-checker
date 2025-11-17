#include "First_Part12.h"
#include "checkDRC.h"
#include "Calculator.h"
int main() {
    int DRC = 0;
    
    vector<string> DRC_pairs;
    map<string, vector<shared_ptr<Node>>> layersSeg;
    vector<Padinfo> pads;
    vector<string> layerName;
    KiCadParser parser;
    map<int, string> NetNamewithID;
    
    std::string filename = "./3.kicad_pcb"; 
    if (parser.parseFile(filename)) {
        std::cout << "解析成功！" << std::endl;
    } else {
        std::cerr << "解析失败！" << std::endl;
        return 0;
    }
    vector<shared_ptr<Node>> footprints = parser.getAllFootprints();
    //vias = parser.getAllVias();  // 获取所有 vias
    vector<shared_ptr<Node>> segments = parser.getAllSegments();
    std::vector<NetInfo> nets = parser.getBoardNets(); 
    auto absCoor = calculateFootprintCoordinates(filename);
    for (auto net:nets){
        NetNamewithID.insert({net.id,net.name});
        //std::cout<<net.id<<" "<<net.name;
    }
    for (auto layerstruct:parser.getBoardLayers()){
        if (layerstruct.name.find(".Cu") != std::string::npos){
            layersSeg.insert({layerstruct.name, vector<shared_ptr<Node>>()});
            layerName.push_back(layerstruct.name);
        }
    }
    for (auto &segment : segments){
        for (std::shared_ptr<Node> &child : segment->children){
            if (child->name == "layer" || child->name == "layers"){
                for (std::string layer: child -> parameters){
                    if (layer == "*.Cu"){
                        for (int i = 0; i < layersSeg.size(); i++){
                            layersSeg.at(layerName[i]).push_back(segment);
                        }
                        break;
                    }

                    if (layer.find(".Cu") != std::string::npos){
                        layersSeg.at(layer).push_back(segment);
                    }
                }
                break;
            }
        }
    }

    for (auto &fp : footprints){
        for (auto child:fp -> children){
            if (child->name == "pad"){
                double pad_grid_x = FindPadAbsoluteCoor(stod(fp -> children[2]->parameters[0]), stod(fp -> children[2]->parameters[1]), child->parameters[0],absCoor)[0];
                double pad_grid_y = FindPadAbsoluteCoor(stod(fp -> children[2]->parameters[0]), stod(fp -> children[2]->parameters[1]), child->parameters[0],absCoor)[1];
                Padinfo pad;
                float padradius;
                pad.x = pad_grid_x;
                pad.y = pad_grid_y;
                float padwidth;
                float padheight;
                int padangle = 0;
                bool samelayer = 0;
                string padnet;
                vector<vector<Vec2>> padboundaries;
                for(auto Child :child->children){
                    if (Child->name == "layer" || Child->name == "layers"){
                        for (std::string layer: Child -> parameters){
                            if(layer == "*.Cu" || layer.find(".Cu") != std::string::npos){  
                                pad.layers.push_back(layer);
                            }
                        } 

                    } else if (Child->name == "size"){
                        padwidth = std::stod(Child->parameters[0]);
                        padheight = std::stod(Child->parameters[1]);
                        padradius = std::stod(Child->parameters[0])/2;
                        pad.width = padwidth;
                        pad.height = padheight;
                        pad.radius = padradius;
                    } else if (Child->name == "at"){
                        if (Child -> parameters.size() > 2){
                            padangle = std::stoi(Child->parameters[2]);
                        }
                    } else if (Child->name == "net"){
                        pad.net = Child->parameters[0];
                    }
                    std::string pad_shape = child -> parameters[child->parameters.size()-1];
                    if (pad_shape == "rect"){
                        pad.boundary = FindRectBoundarys(pad_grid_x, pad_grid_y, padwidth, padheight, padangle);                          
                    } else if (pad_shape=="circle"){
                        pad.boundary = FindCircleBoundarys(pad_grid_x, pad_grid_y, padradius);
                    } else if (pad_shape=="roundrect"){
                        float rratio =0.0;
                        for(auto Child :child->children){
                            if (Child->name == "roundrect_rratio"){
                                rratio= std::stof(Child->parameters[0]);
                            }
                        }
                        pad.boundary = FindRoundrectBoundarys(pad_grid_x, pad_grid_y, padwidth, padheight, rratio, padangle);
                    
                    } else if (pad_shape=="oval"){
                        pad.boundary = FindOvalBoundarys(pad_grid_x, pad_grid_y, padwidth, padheight, padangle);
                    }

                }
                if (pad.net == ""){
                    cerr << "Pad at ("<< pad.x<<","<<pad.y<<") has no net assigned!" << std::endl;
                    return 0;
                }
                pads.push_back(pad);
                cout<<"Pad at ("<<pad.x<<","<<pad.y<<") "<<" of net "<<NetNamewithID.at(stoi(pad.net))<<std::endl;
                cout<<"  Boundary points: ";
                for (auto &boundary : pad.boundary){
                    cout<<"   ";
                    for (auto &point : boundary){
                        cout<<"("<<point.x<<","<<point.y<<") ";
                    }
                    cout<<std::endl;
                }
            }
        }

    }
    for(auto &layerpair : layersSeg){
        for(auto it = layerpair.second.rbegin(); it != layerpair.second.rend(); ++it){
            auto segment = *it;
            double x1, y1, x2, y2;
            float width;
            string net;
            for (std::shared_ptr<Node> &child : segment->children){
                if (child->name == "start"){
                    x1 = std::stod(child->parameters[0]);
                    y1 = std::stod(child->parameters[1]);
                } else if (child->name == "end"){
                    x2 = std::stod(child->parameters[0]);
                    y2 = std::stod(child->parameters[1]);
                } else if (child->name == "width"){
                    width = std::stof(child->parameters[0]);
                } else if (child->name == "net"){
                    net = child->parameters[0];
                }
            }
            auto boundaries = FindSegmentboundarys(x1, y1, x2, y2, width);
            for(auto it1 = it; it1 != layerpair.second.rend(); ++it1){
                //if (it == it1) continue;
                auto otherSegment = *it1;
                double ox1, oy1, ox2, oy2;
                float owidth;
                string onet;
                vector<vector<Vec2>> other_boundaries;
                for (std::shared_ptr<Node> &child : otherSegment->children){
                    if (child->name == "start"){
                        ox1 = std::stod(child->parameters[0]);
                        oy1 = std::stod(child->parameters[1]);
                    } else if (child->name == "end"){
                        ox2 = std::stod(child->parameters[0]);
                        oy2 = std::stod(child->parameters[1]);
                    } else if (child->name == "net"){
                        if (child->parameters[0] == net){
                            goto next_iteration;
                        }
                        onet = child->parameters[0];
                    } else if (child->name == "width"){
                        owidth = std::stof(child->parameters[0]);
                    }
                }
                if(ox1<=min(x1,x2)-width/2-owidth/2 && ox2<=min(x1,x2)-width/2-owidth/2) continue;
                if(ox1>=max(x1,x2)+width/2+owidth/2 && ox2>=max(x1,x2)+width/2+owidth/2) continue;
                if(oy1<=min(y1,y2)-width/2-owidth/2 && oy2<=min(y1,y2)-width/2-owidth/2) continue;
                if(oy1>=max(y1,y2)+width/2+owidth/2 && oy2>=max(y1,y2)+width/2+owidth/2) continue;
                other_boundaries = FindSegmentboundarys(ox1, oy1, ox2, oy2, owidth);
                for(auto &boundary : boundaries){
                    for (auto &other_boundary : other_boundaries){
                        if (boundary.size() == 2 && other_boundary.size() == 2){
                            if(SegmentIntersectsSegment(boundary[0], boundary[1], other_boundary[0], other_boundary[1])){
                                DRC++;
                                DRC_pairs.push_back("DRC Violation between nets " + NetNamewithID.at(stoi(net)) + " and " + NetNamewithID.at(stoi(onet)) + " on layer " + layerpair.first);
                                goto next_iteration;
                            }

                        }  
                        if (boundary.size() == 2 && other_boundary.size() == 3){
                            if(SegmentIntersectsArc(boundary[0], boundary[1], other_boundary[0], other_boundary[1], other_boundary[2])){
                                DRC++;
                                DRC_pairs.push_back("DRC Violation between nets " + NetNamewithID.at(stoi(net)) + " and " + NetNamewithID.at(stoi(onet)) + " on layer " + layerpair.first);
                                goto next_iteration;
                            }  
                        }
                        if (boundary.size() == 3 && other_boundary.size() == 2){
                            if(SegmentIntersectsArc(other_boundary[0], other_boundary[1], boundary[0], boundary[1], boundary[2])){
                                DRC++;
                                DRC_pairs.push_back("DRC Violation between nets " + NetNamewithID.at(stoi(net)) + " and " + NetNamewithID.at(stoi(onet)) + " on layer " + layerpair.first);
                                goto next_iteration;
                            }  
                        }
                        if (boundary.size() == 3 && other_boundary.size() == 3){
                            if(ArcIntersectsArc(boundary[0], boundary[1], boundary[2], other_boundary[0], other_boundary[1], other_boundary[2])){
                                DRC++;
                                DRC_pairs.push_back("DRC Violation between nets " + NetNamewithID.at(stoi(net)) + " and " + NetNamewithID.at(stoi(onet)) + " on layer " + layerpair.first);
                                goto next_iteration;
                            }  
                        }
                    }
                }
                next_iteration:;
            }

            for(auto &pad :pads){
                if (pad.net == net) continue;
                bool samelayer = 0;
                for (auto &padlayer : pad.layers){
                    if (padlayer == layerpair.first || padlayer == "*.Cu"){
                        samelayer = 1;
                        break;
                    }
                }
                if (!samelayer) continue;
                if (x1 < pad.x - max(pad.width,pad.height) - width/2&& x2 < pad.x - max(pad.width,pad.height)- width/2) continue;
                if (x1 > pad.x + max(pad.width,pad.height) + width/2&& x2 > pad.x + max(pad.width,pad.height)+ width/2) continue;
                if (y1 < pad.y - max(pad.width,pad.height) - width/2&& y2 < pad.y - max(pad.width,pad.height)- width/2) continue;
                if (y1 > pad.y + max(pad.width,pad.height) + width/2&& y2 > pad.y + max(pad.width,pad.height)+ width/2) continue;
                for(auto &boundary : boundaries){
                    for (auto &padboundary : pad.boundary){
                        if (boundary.size() == 2 && padboundary.size() == 2){
                            if(SegmentIntersectsSegment(boundary[0], boundary[1], padboundary[0], padboundary[1])){
                                DRC++;
                                DRC_pairs.push_back("DRC Violation between net " + NetNamewithID.at(stoi(net)) + " and pad of net " +  NetNamewithID.at(stoi(pad.net)) + " at" + "(" + std::to_string(pad.x) +","+ std::to_string(pad.y) + ")" + " on layer " + layerpair.first);
                                goto pad_next_iteration;
                            }
                        }
                        if (boundary.size() == 2 && padboundary.size() == 3){
                            if(SegmentIntersectsArc(boundary[0], boundary[1], padboundary[0], padboundary[1], padboundary[2])){
                                DRC++;
                                DRC_pairs.push_back("DRC Violation between net " + NetNamewithID.at(stoi(net)) + " and pad of net " +  NetNamewithID.at(stoi(pad.net)) + " at" + "(" + std::to_string(pad.x) +","+ std::to_string(pad.y) + ")" + " on layer " + layerpair.first);
                                goto pad_next_iteration;
                            }  
                        }
                        if (boundary.size() == 3 && padboundary.size() == 2){
                            if(SegmentIntersectsArc(padboundary[0], padboundary[1], boundary[0], boundary[1], boundary[2])){
                                DRC++;
                                DRC_pairs.push_back("DRC Violation between net " + NetNamewithID.at(stoi(net)) + " and pad of net " +  NetNamewithID.at(stoi(pad.net)) + " at" + "(" + std::to_string(pad.x) +","+ std::to_string(pad.y) + ")" + " on layer " + layerpair.first);
                                goto pad_next_iteration;
                            }
                        }
                        if( boundary.size() == 3 && padboundary.size() == 3){
                            if(ArcIntersectsArc(boundary[0], boundary[1], boundary[2], padboundary[0], padboundary[1], padboundary[2])){
                                DRC++;
                                DRC_pairs.push_back("DRC Violation between net " + NetNamewithID.at(stoi(net)) + " and pad of net " +  NetNamewithID.at(stoi(pad.net)) + " at" + "(" + std::to_string(pad.x) +","+ std::to_string(pad.y) + ")" + " on layer " + layerpair.first);
                                goto pad_next_iteration;
                            }  
                        }
                    }
                }
                pad_next_iteration:;
            } 
        }
    }

    for(auto it = pads.rbegin(); it != pads.rend(); ++it){
        auto pad1 = *it;
        for(auto it1 = it; it1 != pads.rend(); ++it1){
            if (it == it1) continue;
            auto pad2 = *it1;
            if (pad1.net == pad2.net) continue;
            bool samelayer = 0;
            for (auto &padlayer1 : pad1.layers){
                for (auto &padlayer2 : pad2.layers){
                    if (padlayer1 == padlayer2 || padlayer1 == "*.Cu" || padlayer2 == "*.Cu"){
                        samelayer = 1;
                        break;
                    }
                }
                if (samelayer) break;
            }
            if (!samelayer) continue;
            if (pad1.x < pad2.x - max(pad2.width,pad2.height) - max(pad1.width,pad1.height)) continue;
            if (pad1.x > pad2.x + max(pad2.width,pad2.height) + max(pad1.width,pad1.height)) continue;
            if (pad1.y < pad2.y - max(pad2.width,pad2.height) - max(pad1.width,pad1.height)) continue;
            if (pad1.y > pad2.y + max(pad2.width,pad2.height) + max(pad1.width,pad1.height)) continue;
            for(auto &boundary1 : pad1.boundary){
                for (auto &boundary2 : pad2 .boundary){
                    if (boundary1.size() == 2 && boundary2.size() == 2){
                        if(SegmentIntersectsSegment(boundary1[0], boundary1[1], boundary2[0], boundary2[1])){
                            DRC++;
                            DRC_pairs.push_back("DRC Violation between pad of net " + NetNamewithID.at(stoi(pad1.net)) + " and pad of net " +  NetNamewithID.at(stoi(pad2.net)));
                            goto padpair_next_iteration;
                        }
                    }
                    if (boundary1.size() == 2 && boundary2.size() == 3){
                        if(SegmentIntersectsArc(boundary1[0], boundary1[1], boundary2[0], boundary2[1], boundary2[2])){
                            DRC++;
                            DRC_pairs.push_back("DRC Violation between pad of net " + NetNamewithID.at(stoi(pad1.net)) + " and pad of net " +  NetNamewithID.at(stoi(pad2.net)));
                            goto padpair_next_iteration;
                        }
                    }
                    if( boundary1.size() == 3 && boundary2.size() == 2){
                        if(SegmentIntersectsArc(boundary2[0], boundary2[1], boundary1[0], boundary1[1], boundary1[2])){
                            DRC++;
                            DRC_pairs.push_back("DRC Violation between pad of net " + NetNamewithID.at(stoi(pad1.net)) + " and pad of net " +  NetNamewithID.at(stoi(pad2.net)));
                            goto padpair_next_iteration;
                        }
                    }
                    if( boundary1.size() == 3 && boundary2.size() == 3){
                        if(ArcIntersectsArc(boundary1[0], boundary1[1], boundary1[2], boundary2[0], boundary2[1], boundary2[2])){
                            DRC++;
                            DRC_pairs.push_back("DRC Violation between pad of net " + NetNamewithID.at(stoi(pad1.net)) + " and pad of net " +  NetNamewithID.at(stoi(pad2.net)));
                            goto padpair_next_iteration;
                        }
                    }
                }
            }
            padpair_next_iteration:;
        }
    }

            
                
        
    
    for (auto &pair : DRC_pairs){
        std::cout << pair << std::endl;
    }
    printf("DRC Violations Found: %d\n", DRC);
    return 0;
}