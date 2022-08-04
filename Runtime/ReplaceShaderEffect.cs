using UnityEngine;
using UnityEngine.Rendering;

[ExecuteInEditMode]
[RequireComponent(typeof(Camera))]
public class ReplaceShaderEffect : MonoBehaviour
{
    public enum ReplacementModes {
		ObjectId 			= 0,
		CatergoryId			= 1,
		DepthCompressed		= 2,
		DepthMultichannel	= 3,
		Normals				= 4
	};
    public Shader ReplacementShader;
    public ReplacementModes replacementMode = ReplacementModes.DepthCompressed;
    private Camera Cam;
    private Material Mat;

    // TODO: detect change to the dropdown menu and update the shader accordingly
    // instead of in OnEnable().
    void OnEnable()
    {
        if (ReplacementShader == null)
        {
            ReplacementShader = Shader.Find("Hidden/UberReplacement");
        }
        if (ReplacementShader != null)
        {
            Cam  = GetComponent<Camera>();
            var cb = new CommandBuffer();
            cb.SetGlobalFloat("_OutputMode", (int) replacementMode); // The output mode is set to replacementMode
            Cam.AddCommandBuffer(CameraEvent.BeforeForwardOpaque, cb);
            Cam.AddCommandBuffer(CameraEvent.BeforeFinalPass, cb);
            Cam.SetReplacementShader(ReplacementShader, "");
            Cam.clearFlags = CameraClearFlags.SolidColor;
        }
    }

    void OnDisable()
    {
        GetComponent<Camera>().ResetReplacementShader();
    }

    // private void Update()
    // {
    //     if (Cam == null)
    //     {
    //         Cam = this.GetComponent<Camera>();
    //         Cam.depthTextureMode = DepthTextureMode.DepthNormals;
    //     }
        
    //     if (Mat == null)
    //     {
    //         Mat = new Material(ReplacementShader);
    //     }
    // }

    // private void OnPreRender()
    // {
    //     Shader.SetGlobalMatrix(Shader.PropertyToID("UNITY_MATRIX_IV"), Cam.cameraToWorldMatrix);
    // }    

    // private void OnRenderImage(Texture src, RenderTexture dest)
    // {
    //     if (Mat != null)
    //     {
    //         RenderTexture.active = dest;
    //         Graphics.Blit(src, dest, Mat);
    //     } else
    //     {
    //         Graphics.Blit(src, dest);
    //     }
    // }
}
