import open3d as o3d
import numpy as np
import copy

# ================= ÅäÖÃÇøÓò =================
# ÊäÈëÎÄ¼þÃû (ÄãµÄÍáµØÍ¼)
input_file = "scans.pcd" 
# Êä³öÎÄ¼þÃû (°ÚÕýºóµÄµØÍ¼)
output_file = "scans_corrected.pcd"

# ÕâÀïÌîÄãÔÚ TF ÀïÓÃµÄÄÇ¸ö½Ç¶È
# Èç¹ûÄãÊÇÑöÊÓ (Ì§Í·) °²×°£¬TF ÀïÌîµÄÊÇ -0.785 (¸ºÊý)
# ÄÇÃ´ÕâÀïÍ¨³£ÒªÌî "Ïà·´Êý" À´°ÑËü×ª»ØÈ¥£¬»òÕßÌîÒ»ÑùµÄ£¬È¡¾öÓÚ×ø±êÏµ¶¨Òå
# ½¨Òé£ºÏÈÌî -45 ¶ÈÊÔÊÔ£¬Èç¹û¸üÍáÁË£¬¾Í¸Ä³É +45 ¶È
pitch_angle_degree = 50.0  

# ===========================================

def rotate_pcd():
    print(f"ÕýÔÚ¶ÁÈ¡ {input_file} ...")
    pcd = o3d.io.read_point_cloud(input_file)
    
    # ½«½Ç¶È×ª»»Îª»¡¶È
    angle_rad = np.deg2rad(pitch_angle_degree)
    
    # Éú³ÉÐý×ª¾ØÕó (ÈÆ Y ÖáÐý×ª£¬¼´ Pitch)
    # ÕâÀïµÄ¾ØÕó¶ÔÓ¦£ºx, y, z -> ÈÆ Y Öá
    R = pcd.get_rotation_matrix_from_xyz((0, angle_rad, 0))
    
    # Ö´ÐÐÐý×ª
    # center=(0,0,0) ±íÊ¾ÈÆ×ÅÔ­µãÐý×ª£¬¶ø²»ÊÇÈÆ×ÅµãÔÆÖÐÐÄÐý×ª
    pcd.rotate(R, center=(0, 0, 0))
    
    print(f"ÕýÔÚ±£´æµ½ {output_file} ...")
    o3d.io.write_point_cloud(output_file, pcd)
    print("Íê³É£¡ÇëÓÃ Rviz ²é¿´ÐÂµÄ pcd ÎÄ¼þ¡£")

if __name__ == "__main__":
    rotate_pcd()
