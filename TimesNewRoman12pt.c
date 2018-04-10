#include "DAACEDfont.h"
/* 
**  Font data for Times New Roman 12pt
*/

/* Character bitmaps for Times New Roman 12pt */
const uint_8 timesNewRoman_12ptBitmaps[] = 
{
	/* @0 ' ' (6 pixels wide) */
	0x00, 0x00, //                 
	0x00, 0x00, //                 
	0x00, 0x00, //                 
	0x00, 0x00, //                 
	0x00, 0x00, //                 
	0x00, 0x00, //                 

	/* @12 '!' (1 pixels wide) */
	0x0B, 0xFE, //     # ######### 

	/* @14 '"' (3 pixels wide) */
	0x00, 0x1E, //            #### 
	0x00, 0x00, //                 
	0x00, 0x1E, //            #### 

	/* @20 '#' (8 pixels wide) */
	0x01, 0x20, //        #  #     
	0x0F, 0x20, //     ####  #     
	0x01, 0xF0, //        #####    
	0x01, 0x2E, //        #  # ### 
	0x0F, 0x20, //     ####  #     
	0x01, 0xF0, //        #####    
	0x01, 0x2E, //        #  # ### 
	0x01, 0x20, //        #  #     

	/* @36 '$' (6 pixels wide) */
	0x06, 0x1C, //      ##    ###  
	0x08, 0x22, //     #     #   # 
	0x1F, 0xFF, //    #############
	0x08, 0x42, //     #    #    # 
	0x08, 0x82, //     #   #     # 
	0x07, 0x0C, //      ###    ##  

	/* @48 '%' (12 pixels wide) */
	0x00, 0x3C, //           ####  
	0x00, 0x42, //          #    # 
	0x0C, 0x42, //     ##   #    # 
	0x02, 0x3C, //       #   ####  
	0x01, 0x00, //        #        
	0x00, 0xC0, //         ##      
	0x00, 0x20, //           #     
	0x00, 0x10, //            #    
	0x07, 0x8C, //      ####   ##  
	0x08, 0x42, //     #    #    # 
	0x08, 0x40, //     #    #      
	0x07, 0x80, //      ####       

	/* @72 '&' (11 pixels wide) */
	0x07, 0x00, //      ###        
	0x0C, 0x80, //     ##  #       
	0x08, 0x5C, //     #    # ###  
	0x08, 0x62, //     #    ##   # 
	0x08, 0xE2, //     #   ###   # 
	0x07, 0x12, //      ###   #  # 
	0x06, 0x1C, //      ##    ###  
	0x09, 0xA0, //     #  ## #     
	0x08, 0x60, //     #    ##     
	0x08, 0x20, //     #     #     
	0x04, 0x00, //      #          

	/* @94 ''' (1 pixels wide) */
	0x00, 0x1E, //            #### 

	/* @96 '(' (4 pixels wide) */
	0x07, 0xE0, //      ######     
	0x18, 0x18, //    ##      ##   
	0x20, 0x04, //   #          #  
	0x40, 0x02, //  #            # 

	/* @104 ')' (4 pixels wide) */
	0x40, 0x02, //  #            # 
	0x20, 0x04, //   #          #  
	0x18, 0x18, //    ##      ##   
	0x07, 0xE0, //      ######     

	/* @112 '*' (7 pixels wide) */
	0x00, 0x24, //           #  #  
	0x00, 0x24, //           #  #  
	0x00, 0x18, //            ##   
	0x00, 0x7E, //          ###### 
	0x00, 0x18, //            ##   
	0x00, 0x24, //           #  #  
	0x00, 0x24, //           #  #  

	/* @126 '+' (9 pixels wide) */
	0x00, 0x40, //          #      
	0x00, 0x40, //          #      
	0x00, 0x40, //          #      
	0x00, 0x40, //          #      
	0x07, 0xFC, //      #########  
	0x00, 0x40, //          #      
	0x00, 0x40, //          #      
	0x00, 0x40, //          #      
	0x00, 0x40, //          #      

	/* @144 ',' (2 pixels wide) */
	0x28, 0x00, //   # #           
	0x18, 0x00, //    ##           

	/* @148 '-' (4 pixels wide) */
	0x01, 0x00, //        #        
	0x01, 0x00, //        #        
	0x01, 0x00, //        #        
	0x01, 0x00, //        #        

	/* @156 '.' (1 pixels wide) */
	0x08, 0x00, //     #           

	/* @158 '/' (4 pixels wide) */
	0x0C, 0x00, //     ##          
	0x03, 0xC0, //       ####      
	0x00, 0x38, //           ###   
	0x00, 0x06, //              ## 

	/* @166 '0' (6 pixels wide) */
	0x03, 0xF8, //       #######   
	0x04, 0x04, //      #       #  
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x04, 0x04, //      #       #  
	0x03, 0xF8, //       #######   

	/* @178 '1' (4 pixels wide) */
	0x00, 0x04, //              #  
	0x08, 0x04, //     #        #  
	0x0F, 0xFE, //     ########### 
	0x08, 0x00, //     #           

	/* @186 '2' (6 pixels wide) */
	0x0C, 0x0C, //     ##      ##  
	0x0A, 0x02, //     # #       # 
	0x09, 0x02, //     #  #      # 
	0x08, 0x82, //     #   #     # 
	0x08, 0x44, //     #    #   #  
	0x0C, 0x38, //     ##    ###   

	/* @198 '3' (6 pixels wide) */
	0x08, 0x04, //     #        #  
	0x08, 0x02, //     #         # 
	0x08, 0x42, //     #    #    # 
	0x08, 0x62, //     #    ##   # 
	0x04, 0x52, //      #   # #  # 
	0x03, 0x8C, //       ###   ##  

	/* @210 '4' (7 pixels wide) */
	0x01, 0x80, //        ##       
	0x01, 0x40, //        # #      
	0x01, 0x20, //        #  #     
	0x01, 0x18, //        #   ##   
	0x01, 0x04, //        #     #  
	0x0F, 0xFE, //     ########### 
	0x01, 0x00, //        #        

	/* @224 '5' (6 pixels wide) */
	0x08, 0x10, //     #      #    
	0x08, 0x1C, //     #      ###  
	0x08, 0x12, //     #      #  # 
	0x08, 0x12, //     #      #  # 
	0x04, 0x22, //      #    #   # 
	0x03, 0xC2, //       ####    # 

	/* @236 '6' (6 pixels wide) */
	0x03, 0xE0, //       #####     
	0x04, 0x58, //      #   # ##   
	0x08, 0x24, //     #     #  #  
	0x08, 0x22, //     #     #   # 
	0x04, 0x42, //      #   #    # 
	0x03, 0x82, //       ###     # 

	/* @248 '7' (7 pixels wide) */
	0x00, 0x04, //              #  
	0x00, 0x02, //               # 
	0x00, 0x02, //               # 
	0x0C, 0x02, //     ##        # 
	0x03, 0x82, //       ###     # 
	0x00, 0x72, //          ###  # 
	0x00, 0x0E, //             ### 

	/* @262 '8' (6 pixels wide) */
	0x07, 0x1C, //      ###   ###  
	0x08, 0xA2, //     #   # #   # 
	0x08, 0x42, //     #    #    # 
	0x08, 0x42, //     #    #    # 
	0x08, 0xA2, //     #   # #   # 
	0x07, 0x1C, //      ###   ###  

	/* @274 '9' (6 pixels wide) */
	0x08, 0x38, //     #     ###   
	0x08, 0x46, //     #    #   ## 
	0x04, 0x82, //      #  #     # 
	0x04, 0x82, //      #  #     # 
	0x03, 0x44, //       ## #   #  
	0x00, 0xF8, //         #####   

	/* @286 ':' (1 pixels wide) */
	0x08, 0x20, //     #     #     

	/* @288 ';' (2 pixels wide) */
	0x28, 0x20, //   # #     #     
	0x18, 0x00, //    ##           

	/* @292 '<' (7 pixels wide) */
	0x00, 0x40, //          #      
	0x00, 0xA0, //         # #     
	0x00, 0xA0, //         # #     
	0x01, 0x10, //        #   #    
	0x01, 0x10, //        #   #    
	0x01, 0x10, //        #   #    
	0x02, 0x08, //       #     #   

	/* @306 '=' (9 pixels wide) */
	0x01, 0x20, //        #  #     
	0x01, 0x20, //        #  #     
	0x01, 0x20, //        #  #     
	0x01, 0x20, //        #  #     
	0x01, 0x20, //        #  #     
	0x01, 0x20, //        #  #     
	0x01, 0x20, //        #  #     
	0x01, 0x20, //        #  #     
	0x01, 0x20, //        #  #     

	/* @324 '>' (7 pixels wide) */
	0x02, 0x08, //       #     #   
	0x01, 0x10, //        #   #    
	0x01, 0x10, //        #   #    
	0x01, 0x10, //        #   #    
	0x00, 0xA0, //         # #     
	0x00, 0xA0, //         # #     
	0x00, 0x40, //          #      

	/* @338 '?' (5 pixels wide) */
	0x00, 0x0C, //             ##  
	0x00, 0x02, //               # 
	0x0B, 0x82, //     # ###     # 
	0x00, 0x62, //          ##   # 
	0x00, 0x1C, //            ###  

	/* @348 '@' (14 pixels wide) */
	0x0F, 0xC0, //     ######      
	0x10, 0x30, //    #      ##    
	0x20, 0x08, //   #         #   
	0x4F, 0x04, //  #  ####     #  
	0x88, 0xC4, // #   #   ##   #  
	0x88, 0x22, // #   #     #   # 
	0x84, 0x12, // #    #     #  # 
	0x8F, 0x12, // #   ####   #  # 
	0x89, 0xE2, // #   #  ####   # 
	0x88, 0x32, // #   #     ##  # 
	0x44, 0x04, //  #   #       #  
	0x26, 0x08, //   #  ##     #   
	0x11, 0xF0, //    #   #####    
	0x08, 0x00, //     #           

	/* @376 'A' (11 pixels wide) */
	0x08, 0x00, //     #           
	0x08, 0x00, //     #           
	0x0E, 0x00, //     ###         
	0x09, 0xC0, //     #  ###      
	0x01, 0x38, //        #  ###   
	0x01, 0x06, //        #     ## 
	0x01, 0x38, //        #  ###   
	0x09, 0xC0, //     #  ###      
	0x0E, 0x00, //     ###         
	0x08, 0x00, //     #           
	0x08, 0x00, //     #           

	/* @398 'B' (9 pixels wide) */
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x0F, 0xFE, //     ########### 
	0x08, 0x42, //     #    #    # 
	0x08, 0x42, //     #    #    # 
	0x08, 0x42, //     #    #    # 
	0x08, 0x64, //     #    ##  #  
	0x04, 0x98, //      #  #  ##   
	0x03, 0x00, //       ##        

	/* @416 'C' (9 pixels wide) */
	0x01, 0xF0, //        #####    
	0x02, 0x08, //       #     #   
	0x04, 0x04, //      #       #  
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x04, 0x04, //      #       #  
	0x02, 0x1E, //       #    #### 

	/* @434 'D' (10 pixels wide) */
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x0F, 0xFE, //     ########### 
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x04, 0x04, //      #       #  
	0x02, 0x08, //       #     #   
	0x01, 0xF0, //        #####    

	/* @454 'E' (9 pixels wide) */
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x0F, 0xFE, //     ########### 
	0x08, 0x42, //     #    #    # 
	0x08, 0x42, //     #    #    # 
	0x08, 0x42, //     #    #    # 
	0x08, 0xE2, //     #   ###   # 
	0x0C, 0x06, //     ##       ## 
	0x02, 0x00, //       #         

	/* @472 'F' (8 pixels wide) */
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x0F, 0xFE, //     ########### 
	0x08, 0x42, //     #    #    # 
	0x08, 0x42, //     #    #    # 
	0x00, 0x42, //          #    # 
	0x00, 0xE2, //         ###   # 
	0x00, 0x06, //              ## 

	/* @488 'G' (10 pixels wide) */
	0x01, 0xF0, //        #####    
	0x02, 0x08, //       #     #   
	0x04, 0x04, //      #       #  
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x08, 0x42, //     #    #    # 
	0x08, 0x44, //     #    #   #  
	0x07, 0xCE, //      #####  ### 
	0x00, 0x40, //          #      

	/* @508 'H' (11 pixels wide) */
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x0F, 0xFE, //     ########### 
	0x08, 0x42, //     #    #    # 
	0x08, 0x42, //     #    #    # 
	0x00, 0x40, //          #      
	0x08, 0x42, //     #    #    # 
	0x08, 0x42, //     #    #    # 
	0x0F, 0xFE, //     ########### 
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 

	/* @530 'I' (5 pixels wide) */
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x0F, 0xFE, //     ########### 
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 

	/* @540 'J' (6 pixels wide) */
	0x0C, 0x00, //     ##          
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x07, 0xFE, //      ########## 
	0x00, 0x02, //               # 
	0x00, 0x02, //               # 

	/* @552 'K' (11 pixels wide) */
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x0F, 0xFE, //     ########### 
	0x08, 0x42, //     #    #    # 
	0x08, 0x42, //     #    #    # 
	0x00, 0xA0, //         # #     
	0x01, 0x12, //        #   #  # 
	0x0A, 0x0A, //     # #     # # 
	0x0C, 0x06, //     ##       ## 
	0x08, 0x02, //     #         # 
	0x08, 0x00, //     #           

	/* @574 'L' (9 pixels wide) */
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x0F, 0xFE, //     ########### 
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x08, 0x00, //     #           
	0x08, 0x00, //     #           
	0x0C, 0x00, //     ##          
	0x02, 0x00, //       #         

	/* @592 'M' (14 pixels wide) */
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x0F, 0xFE, //     ########### 
	0x08, 0x0C, //     #       ##  
	0x08, 0x30, //     #     ##    
	0x01, 0xC0, //        ###      
	0x02, 0x00, //       #         
	0x0E, 0x00, //     ###         
	0x01, 0x80, //        ##       
	0x08, 0x70, //     #    ###    
	0x08, 0x0C, //     #       ##  
	0x0F, 0xFE, //     ########### 
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 

	/* @620 'N' (12 pixels wide) */
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x0F, 0xFE, //     ########### 
	0x08, 0x08, //     #       #   
	0x08, 0x10, //     #      #    
	0x00, 0x20, //           #     
	0x00, 0xC0, //         ##      
	0x01, 0x02, //        #      # 
	0x02, 0x02, //       #       # 
	0x0F, 0xFE, //     ########### 
	0x00, 0x02, //               # 
	0x00, 0x02, //               # 

	/* @644 'O' (10 pixels wide) */
	0x01, 0xF0, //        #####    
	0x02, 0x08, //       #     #   
	0x04, 0x04, //      #       #  
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x04, 0x04, //      #       #  
	0x02, 0x08, //       #     #   
	0x01, 0xF0, //        #####    

	/* @664 'P' (8 pixels wide) */
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x0F, 0xFE, //     ########### 
	0x08, 0x82, //     #   #     # 
	0x08, 0x82, //     #   #     # 
	0x00, 0x82, //         #     # 
	0x00, 0x44, //          #   #  
	0x00, 0x38, //           ###   

	/* @680 'Q' (10 pixels wide) */
	0x01, 0xF0, //        #####    
	0x02, 0x08, //       #     #   
	0x04, 0x04, //      #       #  
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x18, 0x02, //    ##         # 
	0x28, 0x02, //   # #         # 
	0x24, 0x04, //   #  #       #  
	0x42, 0x08, //  #    #     #   
	0x41, 0xF0, //  #     #####    

	/* @700 'R' (10 pixels wide) */
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x0F, 0xFE, //     ########### 
	0x08, 0x42, //     #    #    # 
	0x08, 0x42, //     #    #    # 
	0x00, 0xC2, //         ##    # 
	0x03, 0x24, //       ##  #  #  
	0x04, 0x18, //      #     ##   
	0x08, 0x00, //     #           
	0x08, 0x00, //     #           

	/* @720 'S' (7 pixels wide) */
	0x0E, 0x18, //     ###    ##   
	0x04, 0x24, //      #    #  #  
	0x08, 0x22, //     #     #   # 
	0x08, 0x42, //     #    #    # 
	0x08, 0x42, //     #    #    # 
	0x04, 0x84, //      #  #    #  
	0x03, 0x0E, //       ##    ### 

	/* @734 'T' (9 pixels wide) */
	0x00, 0x0E, //             ### 
	0x00, 0x02, //               # 
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x0F, 0xFE, //     ########### 
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x00, 0x02, //               # 
	0x00, 0x0E, //             ### 

	/* @752 'U' (11 pixels wide) */
	0x00, 0x02, //               # 
	0x00, 0x02, //               # 
	0x03, 0xFE, //       ######### 
	0x04, 0x02, //      #        # 
	0x08, 0x02, //     #         # 
	0x08, 0x00, //     #           
	0x08, 0x02, //     #         # 
	0x04, 0x02, //      #        # 
	0x03, 0xFE, //       ######### 
	0x00, 0x02, //               # 
	0x00, 0x02, //               # 

	/* @774 'V' (11 pixels wide) */
	0x00, 0x02, //               # 
	0x00, 0x02, //               # 
	0x00, 0x1E, //            #### 
	0x00, 0xE2, //         ###   # 
	0x07, 0x00, //      ###        
	0x08, 0x00, //     #           
	0x07, 0x00, //      ###        
	0x00, 0xE2, //         ###   # 
	0x00, 0x1E, //            #### 
	0x00, 0x02, //               # 
	0x00, 0x02, //               # 

	/* @796 'W' (15 pixels wide) */
	0x00, 0x02, //               # 
	0x00, 0x02, //               # 
	0x00, 0x0E, //             ### 
	0x00, 0x72, //          ###  # 
	0x03, 0x80, //       ###       
	0x0C, 0x02, //     ##        # 
	0x03, 0x82, //       ###     # 
	0x00, 0x5E, //          # #### 
	0x00, 0xE2, //         ###   # 
	0x03, 0x00, //       ##        
	0x0C, 0x00, //     ##          
	0x03, 0xE2, //       #####   # 
	0x00, 0x1E, //            #### 
	0x00, 0x02, //               # 
	0x00, 0x02, //               # 

	/* @826 'X' (11 pixels wide) */
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 
	0x0C, 0x06, //     ##       ## 
	0x0A, 0x0A, //     # #     # # 
	0x01, 0xB2, //        ## ##  # 
	0x00, 0x40, //          #      
	0x01, 0xB0, //        ## ##    
	0x0A, 0x0A, //     # #     # # 
	0x0C, 0x06, //     ##       ## 
	0x08, 0x02, //     #         # 
	0x08, 0x02, //     #         # 

	/* @848 'Y' (10 pixels wide) */
	0x00, 0x02, //               # 
	0x00, 0x02, //               # 
	0x08, 0x0E, //     #       ### 
	0x08, 0x32, //     #     ##  # 
	0x0F, 0xC0, //     ######      
	0x08, 0x20, //     #     #     
	0x08, 0x1A, //     #      ## # 
	0x00, 0x06, //              ## 
	0x00, 0x02, //               # 
	0x00, 0x02, //               # 

	/* @868 'Z' (9 pixels wide) */
	0x08, 0x00, //     #           
	0x0E, 0x0E, //     ###     ### 
	0x09, 0x02, //     #  #      # 
	0x08, 0x82, //     #   #     # 
	0x08, 0x62, //     #    ##   # 
	0x08, 0x12, //     #      #  # 
	0x08, 0x0E, //     #       ### 
	0x0C, 0x02, //     ##        # 
	0x02, 0x00, //       #         

	/* @886 '[' (3 pixels wide) */
	0x7F, 0xFE, //  ############## 
	0x40, 0x02, //  #            # 
	0x40, 0x02, //  #            # 

	/* @892 '\' (4 pixels wide) */
	0x00, 0x06, //              ## 
	0x00, 0x78, //          ####   
	0x03, 0x80, //       ###       
	0x0C, 0x00, //     ##          

	/* @900 ']' (3 pixels wide) */
	0x40, 0x02, //  #            # 
	0x40, 0x02, //  #            # 
	0x7F, 0xFE, //  ############## 

	/* @906 '^' (8 pixels wide) */
	0x00, 0x40, //          #      
	0x00, 0x30, //           ##    
	0x00, 0x0C, //             ##  
	0x00, 0x02, //               # 
	0x00, 0x02, //               # 
	0x00, 0x0C, //             ##  
	0x00, 0x30, //           ##    
	0x00, 0x40, //          #      

	/* @922 '_' (8 pixels wide) */
	0x40, 0x00, //  #              
	0x40, 0x00, //  #              
	0x40, 0x00, //  #              
	0x40, 0x00, //  #              
	0x40, 0x00, //  #              
	0x40, 0x00, //  #              
	0x40, 0x00, //  #              
	0x40, 0x00, //  #              

	/* @938 '`' (2 pixels wide) */
	0x00, 0x02, //               # 
	0x00, 0x0C, //             ##  

	/* @942 'a' (6 pixels wide) */
	0x0E, 0x40, //     ###  #      
	0x09, 0x20, //     #  #  #     
	0x09, 0x20, //     #  #  #     
	0x04, 0xA0, //      #  # #     
	0x0F, 0xC0, //     ######      
	0x08, 0x00, //     #           

	/* @954 'b' (7 pixels wide) */
	0x00, 0x04, //              #  
	0x07, 0xFE, //      ########## 
	0x08, 0x40, //     #    #      
	0x08, 0x20, //     #     #     
	0x08, 0x20, //     #     #     
	0x04, 0x40, //      #   #      
	0x03, 0x80, //       ###       

	/* @968 'c' (5 pixels wide) */
	0x07, 0xC0, //      #####      
	0x08, 0x20, //     #     #     
	0x08, 0x20, //     #     #     
	0x08, 0x20, //     #     #     
	0x04, 0x40, //      #   #      

	/* @978 'd' (7 pixels wide) */
	0x03, 0x80, //       ###       
	0x04, 0x40, //      #   #      
	0x08, 0x20, //     #     #     
	0x08, 0x20, //     #     #     
	0x04, 0x44, //      #   #   #  
	0x0F, 0xFE, //     ########### 
	0x04, 0x00, //      #          

	/* @992 'e' (5 pixels wide) */
	0x07, 0xC0, //      #####      
	0x08, 0xA0, //     #   # #     
	0x08, 0xA0, //     #   # #     
	0x08, 0xA0, //     #   # #     
	0x04, 0xC0, //      #  ##      

	/* @1002 'f' (6 pixels wide) */
	0x08, 0x20, //     #     #     
	0x0F, 0xFC, //     ##########  
	0x08, 0x22, //     #     #   # 
	0x08, 0x22, //     #     #   # 
	0x00, 0x02, //               # 
	0x00, 0x04, //              #  

	/* @1014 'g' (7 pixels wide) */
	0x30, 0x00, //   ##            
	0x4D, 0xC0, //  #  ## ###      
	0x4A, 0x20, //  #  # #   #     
	0x4A, 0x20, //  #  # #   #     
	0x4A, 0x20, //  #  # #   #     
	0x49, 0xE0, //  #  #  ####     
	0x30, 0x20, //   ##      #     

	/* @1028 'h' (7 pixels wide) */
	0x08, 0x04, //     #        #  
	0x0F, 0xFE, //     ########### 
	0x08, 0x40, //     #    #      
	0x00, 0x20, //           #     
	0x08, 0x20, //     #     #     
	0x0F, 0xC0, //     ######      
	0x08, 0x00, //     #           

	/* @1042 'i' (3 pixels wide) */
	0x08, 0x40, //     #    #      
	0x0F, 0xE2, //     #######   # 
	0x08, 0x00, //     #           

	/* @1048 'j' (4 pixels wide) */
	0x40, 0x00, //  #              
	0x40, 0x00, //  #              
	0x40, 0x40, //  #       #      
	0x3F, 0xE2, //   #########   # 

	/* @1056 'k' (8 pixels wide) */
	0x08, 0x04, //     #        #  
	0x0F, 0xFE, //     ########### 
	0x09, 0x00, //     #  #        
	0x01, 0x80, //        ##       
	0x06, 0x60, //      ##  ##     
	0x08, 0x20, //     #     #     
	0x08, 0x20, //     #     #     
	0x08, 0x00, //     #           

	/* @1072 'l' (3 pixels wide) */
	0x08, 0x04, //     #        #  
	0x0F, 0xFE, //     ########### 
	0x08, 0x00, //     #           

	/* @1078 'm' (11 pixels wide) */
	0x08, 0x40, //     #    #      
	0x0F, 0xE0, //     #######     
	0x08, 0x40, //     #    #      
	0x00, 0x20, //           #     
	0x08, 0x20, //     #     #     
	0x0F, 0xC0, //     ######      
	0x08, 0x40, //     #    #      
	0x00, 0x20, //           #     
	0x08, 0x20, //     #     #     
	0x0F, 0xC0, //     ######      
	0x08, 0x00, //     #           

	/* @1100 'n' (7 pixels wide) */
	0x08, 0x40, //     #    #      
	0x0F, 0xE0, //     #######     
	0x08, 0x40, //     #    #      
	0x00, 0x20, //           #     
	0x08, 0x20, //     #     #     
	0x0F, 0xC0, //     ######      
	0x08, 0x00, //     #           

	/* @1114 'o' (6 pixels wide) */
	0x03, 0x80, //       ###       
	0x04, 0x40, //      #   #      
	0x08, 0x20, //     #     #     
	0x08, 0x20, //     #     #     
	0x04, 0x40, //      #   #      
	0x03, 0x80, //       ###       

	/* @1126 'p' (7 pixels wide) */
	0x40, 0x40, //  #       #      
	0x7F, 0xE0, //  ##########     
	0x44, 0x40, //  #   #   #      
	0x08, 0x20, //     #     #     
	0x08, 0x20, //     #     #     
	0x04, 0x40, //      #   #      
	0x03, 0x80, //       ###       

	/* @1140 'q' (7 pixels wide) */
	0x07, 0x80, //      ####       
	0x08, 0x40, //     #    #      
	0x08, 0x20, //     #     #     
	0x08, 0x20, //     #     #     
	0x44, 0x40, //  #   #   #      
	0x7F, 0xE0, //  ##########     
	0x40, 0x00, //  #              

	/* @1154 'r' (5 pixels wide) */
	0x08, 0x40, //     #    #      
	0x0F, 0xE0, //     #######     
	0x08, 0x40, //     #    #      
	0x00, 0x20, //           #     
	0x00, 0x20, //           #     

	/* @1164 's' (4 pixels wide) */
	0x0C, 0xC0, //     ##  ##      
	0x09, 0x20, //     #  #  #     
	0x09, 0x20, //     #  #  #     
	0x06, 0x60, //      ##  ##     

	/* @1172 't' (4 pixels wide) */
	0x00, 0x20, //           #     
	0x0F, 0xF8, //     #########   
	0x08, 0x20, //     #     #     
	0x04, 0x20, //      #    #     

	/* @1180 'u' (7 pixels wide) */
	0x00, 0x20, //           #     
	0x07, 0xE0, //      ######     
	0x08, 0x00, //     #           
	0x08, 0x00, //     #           
	0x04, 0x20, //      #    #     
	0x0F, 0xE0, //     #######     
	0x04, 0x00, //      #          

	/* @1194 'v' (7 pixels wide) */
	0x00, 0x20, //           #     
	0x00, 0xE0, //         ###     
	0x07, 0x20, //      ###  #     
	0x08, 0x00, //     #           
	0x07, 0x20, //      ###  #     
	0x00, 0xE0, //         ###     
	0x00, 0x20, //           #     

	/* @1208 'w' (11 pixels wide) */
	0x00, 0x20, //           #     
	0x00, 0xE0, //         ###     
	0x07, 0x20, //      ###  #     
	0x0C, 0x00, //     ##          
	0x03, 0x20, //       ##  #     
	0x00, 0xE0, //         ###     
	0x07, 0x20, //      ###  #     
	0x0C, 0x00, //     ##          
	0x03, 0x20, //       ##  #     
	0x00, 0xE0, //         ###     
	0x00, 0x20, //           #     

	/* @1230 'x' (7 pixels wide) */
	0x08, 0x20, //     #     #     
	0x0C, 0x60, //     ##   ##     
	0x0A, 0xA0, //     # # # #     
	0x01, 0x00, //        #        
	0x0A, 0xA0, //     # # # #     
	0x0C, 0x60, //     ##   ##     
	0x08, 0x20, //     #     #     

	/* @1244 'y' (7 pixels wide) */
	0x40, 0x20, //  #        #     
	0x40, 0xE0, //  #      ###     
	0x23, 0x20, //   #   ##  #     
	0x1C, 0x00, //    ###          
	0x07, 0x20, //      ###  #     
	0x00, 0xE0, //         ###     
	0x00, 0x20, //           #     

	/* @1258 'z' (6 pixels wide) */
	0x08, 0x00, //     #           
	0x0C, 0x60, //     ##   ##     
	0x0B, 0x20, //     # ##  #     
	0x08, 0xA0, //     #   # #     
	0x08, 0x60, //     #    ##     
	0x0C, 0x20, //     ##    #     

	/* @1270 '{' (4 pixels wide) */
	0x00, 0x80, //         #       
	0x1F, 0x78, //    ##### ####   
	0x20, 0x04, //   #          #  
	0x40, 0x02, //  #            # 

	/* @1278 '|' (3 pixels wide) */
    0x00, 0x00,
	0xFF, 0xFE, // ############### 
    0x00, 0x00,
    
	/* @1284 '}' (4 pixels wide) */
	0x40, 0x02, //  #            # 
	0x20, 0x04, //   #          #  
	0x1F, 0x78, //    ##### ####   
	0x00, 0x80, //         #       

	/* @1292 '~' (9 pixels wide) */
	0x01, 0x00, //        #        
	0x00, 0x80, //         #       
	0x00, 0x80, //         #       
	0x00, 0x80, //         #       
	0x01, 0x00, //        #        
	0x01, 0x00, //        #        
	0x01, 0x00, //        #        
	0x01, 0x00, //        #        
	0x00, 0x80, //         #       
};

/* Character descriptors for Times New Roman 12pt */
/* { [Char width in bits], [Offset into timesNewRoman_12ptCharBitmaps in bytes] } */
const FONT_CHAR_INFO timesNewRoman_12ptDescriptors[] = 
{
	{3, 0}, 		/*   */ 
	{1, 12}, 		/* ! */ 
	{3, 14}, 		/* " */ 
	{8, 20}, 		/* # */ 
	{6, 36}, 		/* $ */ 
	{12, 48}, 		/* % */ 
	{11, 72}, 		/* & */ 
	{1, 94}, 		/* ' */ 
	{4, 96}, 		/* ( */ 
	{4, 104}, 		/* ) */ 
	{7, 112}, 		/* * */ 
	{9, 126}, 		/* + */ 
	{2, 144}, 		/* , */ 
	{4, 148}, 		/* - */ 
	{1, 156}, 		/* . */ 
	{4, 158}, 		/* / */ 
	{6, 166}, 		/* 0 */ 
	{4, 178}, 		/* 1 */ 
	{6, 186}, 		/* 2 */ 
	{6, 198}, 		/* 3 */ 
	{7, 210}, 		/* 4 */ 
	{6, 224}, 		/* 5 */ 
	{6, 236}, 		/* 6 */ 
	{7, 248}, 		/* 7 */ 
	{6, 262}, 		/* 8 */ 
	{6, 274}, 		/* 9 */ 
	{1, 286}, 		/* : */ 
	{2, 288}, 		/* ; */ 
	{7, 292}, 		/* < */ 
	{9, 306}, 		/* = */ 
	{7, 324}, 		/* > */ 
	{5, 338}, 		/* ? */ 
	{14, 348}, 		/* @ */ 
	{11, 376}, 		/* A */ 
	{9, 398}, 		/* B */ 
	{9, 416}, 		/* C */ 
	{10, 434}, 		/* D */ 
	{9, 454}, 		/* E */ 
	{8, 472}, 		/* F */ 
	{10, 488}, 		/* G */ 
	{11, 508}, 		/* H */ 
	{5, 530}, 		/* I */ 
	{6, 540}, 		/* J */ 
	{11, 552}, 		/* K */ 
	{9, 574}, 		/* L */ 
	{14, 592}, 		/* M */ 
	{12, 620}, 		/* N */ 
	{10, 644}, 		/* O */ 
	{8, 664}, 		/* P */ 
	{10, 680}, 		/* Q */ 
	{10, 700}, 		/* R */ 
	{7, 720}, 		/* S */ 
	{9, 734}, 		/* T */ 
	{11, 752}, 		/* U */ 
	{11, 774}, 		/* V */ 
	{15, 796}, 		/* W */ 
	{11, 826}, 		/* X */ 
	{10, 848}, 		/* Y */ 
	{9, 868}, 		/* Z */ 
	{3, 886}, 		/* [ */ 
	{4, 892}, 		/* \ */ 
	{3, 900}, 		/* ] */ 
	{8, 906}, 		/* ^ */ 
	{8, 922}, 		/* _ */ 
	{2, 938}, 		/* ` */ 
	{6, 942}, 		/* a */ 
	{7, 954}, 		/* b */ 
	{5, 968}, 		/* c */ 
	{7, 978}, 		/* d */ 
	{5, 992}, 		/* e */ 
	{6, 1002}, 		/* f */ 
	{7, 1014}, 		/* g */ 
	{7, 1028}, 		/* h */ 
	{3, 1042}, 		/* i */ 
	{4, 1048}, 		/* j */ 
	{8, 1056}, 		/* k */ 
	{3, 1072}, 		/* l */ 
	{11, 1078}, 		/* m */ 
	{7, 1100}, 		/* n */ 
	{6, 1114}, 		/* o */ 
	{7, 1126}, 		/* p */ 
	{7, 1140}, 		/* q */ 
	{5, 1154}, 		/* r */ 
	{4, 1164}, 		/* s */ 
	{4, 1172}, 		/* t */ 
	{7, 1180}, 		/* u */ 
	{7, 1194}, 		/* v */ 
	{11, 1208}, 		/* w */ 
	{7, 1230}, 		/* x */ 
	{7, 1244}, 		/* y */ 
	{6, 1258}, 		/* z */ 
	{4, 1270}, 		/* { */ 
	{3, 1278}, 		/* | */ 
	{4, 1284}, 		/* } */ 
	{9, 1292}, 		/* ~ */ 
};

/* Font information for Times New Roman 12pt */
const FONT_INFO timesNewRoman_12ptFontInfo =
{
	16, /*  Character height */
    2,  /* Character Spacing */
	' ', /*  Start character */
	'~', /*  End character */
	timesNewRoman_12ptDescriptors, /*  Character descriptor array */
	timesNewRoman_12ptBitmaps, /*  Character bitmap array */
    0
};
