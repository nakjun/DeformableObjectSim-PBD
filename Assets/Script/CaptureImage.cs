using Assets.script;
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;
using UnityEngine.Rendering;

public class CaptureImage : MonoBehaviour
{
    public bool writeImage = false;
    public int maxFrame = 400;
    public string imgPath = "E:/Labdata/Unity/PBDDeformation/20230329/";



    int frame = 0;
    void Start()
    {

    }
    void writeImageData(int i, string ImgPath)
    {
        if ((i%200)==0 && i <= maxFrame)
            ScreenCapture.CaptureScreenshot(ImgPath + "frame" + i.ToString().PadLeft(3, '0') + ".png");
    }
    // Update is called once per frame
    void Update()
    {
        // if (writeImage)
        // {
        //     writeImageData(frame, imgPath);
        //     //print(frame);
        //     if (frame == maxFrame - 1)
        //     {
        //         print("Done");
        //         UnityEditor.EditorApplication.isPlaying = false;
        //     }
        // }


        // frame++;
    }
}
