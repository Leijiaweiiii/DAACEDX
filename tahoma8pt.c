#include "DAACEDfont.h"
/* 
**  Font data for Tahoma 8pt
*/

/* Character bitmaps for Tahoma 8pt */
const uint_8 tahoma_8ptBitmaps[] = 
{
	/* @0 ' ' (6 pixels wide) */
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            
	0x00, 0x00, //            

	/* @12 '!' (1 pixels wide) */
	0x2F, 0xC0, //   # ###### 

	/* @14 '"' (3 pixels wide) */
	0x00, 0xE0, //         ###
	0x00, 0x00, //            
	0x00, 0xE0, //         ###

	/* @20 '#' (7 pixels wide) */
	0x08, 0x00, //     #      
	0x39, 0x00, //   ###  #   
	0x0F, 0x00, //     ####   
	0x39, 0xC0, //   ###  ### 
	0x0F, 0x00, //     ####   
	0x09, 0xC0, //     #  ### 
	0x01, 0x00, //        #   

	/* @34 '$' (5 pixels wide) */
	0x23, 0x00, //   #   ##   
	0x24, 0x80, //   #  #  #  
	0xFF, 0xE0, // ###########
	0x24, 0x80, //   #  #  #  
	0x18, 0x80, //    ##   #  

	/* @44 '%' (10 pixels wide) */
	0x01, 0x80, //        ##  
	0x02, 0x40, //       #  # 
	0x02, 0x40, //       #  # 
	0x31, 0x80, //   ##   ##  
	0x0C, 0x00, //     ##     
	0x03, 0x00, //       ##   
	0x18, 0xC0, //    ##   ## 
	0x24, 0x00, //   #  #     
	0x24, 0x00, //   #  #     
	0x18, 0x00, //    ##      

	/* @64 '&' (7 pixels wide) */
	0x1D, 0x80, //    ### ##  
	0x22, 0x40, //   #   #  # 
	0x22, 0x40, //   #   #  # 
	0x25, 0x80, //   #  # ##  
	0x18, 0x00, //    ##      
	0x16, 0x00, //    # ##    
	0x20, 0x00, //   #        

	/* @78 ''' (1 pixels wide) */
	0x00, 0xE0, //         ###

	/* @80 '(' (3 pixels wide) */
	0x1F, 0x00, //    #####   
	0x60, 0xC0, //  ##     ## 
	0x80, 0x20, // #         #

	/* @86 ')' (3 pixels wide) */
	0x80, 0x20, // #         #
	0x60, 0xC0, //  ##     ## 
	0x1F, 0x00, //    #####   

	/* @92 '*' (5 pixels wide) */
	0x01, 0x40, //        # # 
	0x00, 0x80, //         #  
	0x03, 0xE0, //       #####
	0x00, 0x80, //         #  
	0x01, 0x40, //        # # 

	/* @102 '+' (7 pixels wide) */
	0x04, 0x00, //      #     
	0x04, 0x00, //      #     
	0x04, 0x00, //      #     
	0x3F, 0x80, //   #######  
	0x04, 0x00, //      #     
	0x04, 0x00, //      #     
	0x04, 0x00, //      #     

	/* @116 ',' (2 pixels wide) */
	0x80, 0x00, // #          
	0x70, 0x00, //  ###       

	/* @120 '-' (3 pixels wide) */
	0x04, 0x00, //      #     
	0x04, 0x00, //      #     
	0x04, 0x00, //      #     

	/* @126 '.' (1 pixels wide) */
	0x30, 0x00, //   ##       

	/* @128 '/' (3 pixels wide) */
	0xE0, 0x00, // ###        
	0x1F, 0x00, //    #####   
	0x00, 0xE0, //         ###

	/* @134 '0' (5 pixels wide) */
	0x1F, 0x80, //    ######  
	0x20, 0x40, //   #      # 
	0x20, 0x40, //   #      # 
	0x20, 0x40, //   #      # 
	0x1F, 0x80, //    ######  

	/* @144 '1' (3 pixels wide) */
	0x20, 0x80, //   #     #  
	0x3F, 0xC0, //   ######## 
	0x20, 0x00, //   #        

	/* @150 '2' (5 pixels wide) */
	0x30, 0x80, //   ##    #  
	0x28, 0x40, //   # #    # 
	0x24, 0x40, //   #  #   # 
	0x22, 0x40, //   #   #  # 
	0x21, 0x80, //   #    ##  

	/* @160 '3' (5 pixels wide) */
	0x10, 0x80, //    #    #  
	0x20, 0x40, //   #      # 
	0x22, 0x40, //   #   #  # 
	0x22, 0x40, //   #   #  # 
	0x1D, 0x80, //    ### ##  

	/* @170 '4' (5 pixels wide) */
	0x06, 0x00, //      ##    
	0x05, 0x00, //      # #   
	0x04, 0x80, //      #  #  
	0x3F, 0xC0, //   ######## 
	0x04, 0x00, //      #     

	/* @180 '5' (5 pixels wide) */
	0x13, 0xC0, //    #  #### 
	0x22, 0x40, //   #   #  # 
	0x22, 0x40, //   #   #  # 
	0x22, 0x40, //   #   #  # 
	0x1C, 0x40, //    ###   # 

	/* @190 '6' (5 pixels wide) */
	0x1F, 0x00, //    #####   
	0x22, 0x80, //   #   # #  
	0x22, 0x40, //   #   #  # 
	0x22, 0x40, //   #   #  # 
	0x1C, 0x00, //    ###     

	/* @200 '7' (5 pixels wide) */
	0x00, 0x40, //          # 
	0x30, 0x40, //   ##     # 
	0x0C, 0x40, //     ##   # 
	0x03, 0x40, //       ## # 
	0x00, 0xC0, //         ## 

	/* @210 '8' (5 pixels wide) */
	0x1D, 0x80, //    ### ##  
	0x22, 0x40, //   #   #  # 
	0x22, 0x40, //   #   #  # 
	0x22, 0x40, //   #   #  # 
	0x1D, 0x80, //    ### ##  

	/* @220 '9' (5 pixels wide) */
	0x03, 0x80, //       ###  
	0x24, 0x40, //   #  #   # 
	0x24, 0x40, //   #  #   # 
	0x14, 0x40, //    # #   # 
	0x0F, 0x80, //     #####  

	/* @230 ':' (1 pixels wide) */
	0x33, 0x00, //   ##  ##   

	/* @232 ';' (2 pixels wide) */
	0x80, 0x00, // #          
	0x73, 0x00, //  ###  ##   

	/* @236 '<' (6 pixels wide) */
	0x04, 0x00, //      #     
	0x0A, 0x00, //     # #    
	0x0A, 0x00, //     # #    
	0x11, 0x00, //    #   #   
	0x11, 0x00, //    #   #   
	0x20, 0x80, //   #     #  

	/* @248 '=' (7 pixels wide) */
	0x0A, 0x00, //     # #    
	0x0A, 0x00, //     # #    
	0x0A, 0x00, //     # #    
	0x0A, 0x00, //     # #    
	0x0A, 0x00, //     # #    
	0x0A, 0x00, //     # #    
	0x0A, 0x00, //     # #    

	/* @262 '>' (6 pixels wide) */
	0x20, 0x80, //   #     #  
	0x11, 0x00, //    #   #   
	0x11, 0x00, //    #   #   
	0x0A, 0x00, //     # #    
	0x0A, 0x00, //     # #    
	0x04, 0x00, //      #     

	/* @274 '?' (4 pixels wide) */
	0x00, 0x40, //          # 
	0x2C, 0x40, //   # ##   # 
	0x02, 0x40, //       #  # 
	0x01, 0x80, //        ##  

	/* @282 '@' (9 pixels wide) */
	0x1F, 0x00, //    #####   
	0x20, 0x80, //   #     #  
	0x4E, 0x40, //  #  ###  # 
	0x51, 0x40, //  # #   # # 
	0x51, 0x40, //  # #   # # 
	0x5F, 0x40, //  # ##### # 
	0x10, 0x40, //    #     # 
	0x10, 0x80, //    #    #  
	0x0F, 0x00, //     ####   

	/* @300 'A' (6 pixels wide) */
	0x38, 0x00, //   ###      
	0x0F, 0x00, //     ####   
	0x08, 0xC0, //     #   ## 
	0x08, 0xC0, //     #   ## 
	0x0F, 0x00, //     ####   
	0x38, 0x00, //   ###      

	/* @312 'B' (5 pixels wide) */
	0x3F, 0xC0, //   ######## 
	0x22, 0x40, //   #   #  # 
	0x22, 0x40, //   #   #  # 
	0x22, 0x40, //   #   #  # 
	0x1D, 0x80, //    ### ##  

	/* @322 'C' (6 pixels wide) */
	0x0F, 0x00, //     ####   
	0x10, 0x80, //    #    #  
	0x20, 0x40, //   #      # 
	0x20, 0x40, //   #      # 
	0x20, 0x40, //   #      # 
	0x20, 0x40, //   #      # 

	/* @334 'D' (6 pixels wide) */
	0x3F, 0xC0, //   ######## 
	0x20, 0x40, //   #      # 
	0x20, 0x40, //   #      # 
	0x20, 0x40, //   #      # 
	0x10, 0x80, //    #    #  
	0x0F, 0x00, //     ####   

	/* @346 'E' (5 pixels wide) */
	0x3F, 0xC0, //   ######## 
	0x22, 0x40, //   #   #  # 
	0x22, 0x40, //   #   #  # 
	0x22, 0x40, //   #   #  # 
	0x20, 0x40, //   #      # 

	/* @356 'F' (5 pixels wide) */
	0x3F, 0xC0, //   ######## 
	0x02, 0x40, //       #  # 
	0x02, 0x40, //       #  # 
	0x02, 0x40, //       #  # 
	0x02, 0x40, //       #  # 

	/* @366 'G' (6 pixels wide) */
	0x0F, 0x00, //     ####   
	0x10, 0x80, //    #    #  
	0x20, 0x40, //   #      # 
	0x24, 0x40, //   #  #   # 
	0x24, 0x40, //   #  #   # 
	0x3C, 0x40, //   ####   # 

	/* @378 'H' (6 pixels wide) */
	0x3F, 0xC0, //   ######## 
	0x02, 0x00, //       #    
	0x02, 0x00, //       #    
	0x02, 0x00, //       #    
	0x02, 0x00, //       #    
	0x3F, 0xC0, //   ######## 

	/* @390 'I' (3 pixels wide) */
	0x20, 0x40, //   #      # 
	0x3F, 0xC0, //   ######## 
	0x20, 0x40, //   #      # 

	/* @396 'J' (4 pixels wide) */
	0x20, 0x00, //   #        
	0x20, 0x40, //   #      # 
	0x20, 0x40, //   #      # 
	0x1F, 0xC0, //    ####### 

	/* @404 'K' (5 pixels wide) */
	0x3F, 0xC0, //   ######## 
	0x06, 0x00, //      ##    
	0x09, 0x00, //     #  #   
	0x10, 0x80, //    #    #  
	0x20, 0x40, //   #      # 

	/* @414 'L' (4 pixels wide) */
	0x3F, 0xC0, //   ######## 
	0x20, 0x00, //   #        
	0x20, 0x00, //   #        
	0x20, 0x00, //   #        

	/* @422 'M' (7 pixels wide) */
	0x3F, 0xC0, //   ######## 
	0x00, 0xC0, //         ## 
	0x03, 0x00, //       ##   
	0x0C, 0x00, //     ##     
	0x03, 0x00, //       ##   
	0x00, 0xC0, //         ## 
	0x3F, 0xC0, //   ######## 

	/* @436 'N' (6 pixels wide) */
	0x3F, 0xC0, //   ######## 
	0x00, 0xC0, //         ## 
	0x03, 0x00, //       ##   
	0x0C, 0x00, //     ##     
	0x30, 0x00, //   ##       
	0x3F, 0xC0, //   ######## 

	/* @448 'O' (7 pixels wide) */
	0x0F, 0x00, //     ####   
	0x10, 0x80, //    #    #  
	0x20, 0x40, //   #      # 
	0x20, 0x40, //   #      # 
	0x20, 0x40, //   #      # 
	0x10, 0x80, //    #    #  
	0x0F, 0x00, //     ####   

	/* @462 'P' (5 pixels wide) */
	0x3F, 0xC0, //   ######## 
	0x04, 0x40, //      #   # 
	0x04, 0x40, //      #   # 
	0x04, 0x40, //      #   # 
	0x03, 0x80, //       ###  

	/* @472 'Q' (7 pixels wide) */
	0x0F, 0x00, //     ####   
	0x10, 0x80, //    #    #  
	0x20, 0x40, //   #      # 
	0x20, 0x40, //   #      # 
	0x60, 0x40, //  ##      # 
	0x90, 0x80, // #  #    #  
	0x8F, 0x00, // #   ####   

	/* @486 'R' (6 pixels wide) */
	0x3F, 0xC0, //   ######## 
	0x04, 0x40, //      #   # 
	0x04, 0x40, //      #   # 
	0x0C, 0x40, //     ##   # 
	0x13, 0x80, //    #  ###  
	0x20, 0x00, //   #        

	/* @498 'S' (5 pixels wide) */
	0x21, 0x80, //   #    ##  
	0x22, 0x40, //   #   #  # 
	0x22, 0x40, //   #   #  # 
	0x22, 0x40, //   #   #  # 
	0x1C, 0x40, //    ###   # 

	/* @508 'T' (5 pixels wide) */
	0x00, 0x40, //          # 
	0x00, 0x40, //          # 
	0x3F, 0xC0, //   ######## 
	0x00, 0x40, //          # 
	0x00, 0x40, //          # 

	/* @518 'U' (6 pixels wide) */
	0x1F, 0xC0, //    ####### 
	0x20, 0x00, //   #        
	0x20, 0x00, //   #        
	0x20, 0x00, //   #        
	0x20, 0x00, //   #        
	0x1F, 0xC0, //    ####### 

	/* @530 'V' (5 pixels wide) */
	0x01, 0xC0, //        ### 
	0x0E, 0x00, //     ###    
	0x30, 0x00, //   ##       
	0x0E, 0x00, //     ###    
	0x01, 0xC0, //        ### 

	/* @540 'W' (9 pixels wide) */
	0x01, 0xC0, //        ### 
	0x0E, 0x00, //     ###    
	0x30, 0x00, //   ##       
	0x0E, 0x00, //     ###    
	0x01, 0xC0, //        ### 
	0x0E, 0x00, //     ###    
	0x30, 0x00, //   ##       
	0x0E, 0x00, //     ###    
	0x01, 0xC0, //        ### 

	/* @558 'X' (5 pixels wide) */
	0x30, 0xC0, //   ##    ## 
	0x09, 0x00, //     #  #   
	0x06, 0x00, //      ##    
	0x09, 0x00, //     #  #   
	0x30, 0xC0, //   ##    ## 

	/* @568 'Y' (5 pixels wide) */
	0x00, 0xC0, //         ## 
	0x03, 0x00, //       ##   
	0x3C, 0x00, //   ####     
	0x03, 0x00, //       ##   
	0x00, 0xC0, //         ## 

	/* @578 'Z' (5 pixels wide) */
	0x30, 0x40, //   ##     # 
	0x28, 0x40, //   # #    # 
	0x26, 0x40, //   #  ##  # 
	0x21, 0x40, //   #    # # 
	0x20, 0xC0, //   #     ## 

	/* @588 '[' (3 pixels wide) */
	0xFF, 0xE0, // ###########
	0x80, 0x20, // #         #
	0x80, 0x20, // #         #

	/* @594 '\' (3 pixels wide) */
	0x00, 0xE0, //         ###
	0x1F, 0x00, //    #####   
	0xE0, 0x00, // ###        

	/* @600 ']' (3 pixels wide) */
	0x80, 0x20, // #         #
	0x80, 0x20, // #         #
	0xFF, 0xE0, // ###########

	/* @606 '^' (7 pixels wide) */
	0x02, 0x00, //       #    
	0x01, 0x00, //        #   
	0x00, 0x80, //         #  
	0x00, 0x40, //          # 
	0x00, 0x80, //         #  
	0x01, 0x00, //        #   
	0x02, 0x00, //       #    

	/* @620 '_' (6 pixels wide) */
	0x80, 0x00, // #          
	0x80, 0x00, // #          
	0x80, 0x00, // #          
	0x80, 0x00, // #          
	0x80, 0x00, // #          
	0x80, 0x00, // #          

	/* @632 '`' (2 pixels wide) */
	0x00, 0x20, //           #
	0x00, 0x40, //          # 

	/* @636 'a' (5 pixels wide) */
	0x18, 0x00, //    ##      
	0x25, 0x00, //   #  # #   
	0x25, 0x00, //   #  # #   
	0x25, 0x00, //   #  # #   
	0x3E, 0x00, //   #####    

	/* @646 'b' (5 pixels wide) */
	0x3F, 0xE0, //   #########
	0x21, 0x00, //   #    #   
	0x21, 0x00, //   #    #   
	0x21, 0x00, //   #    #   
	0x1E, 0x00, //    ####    

	/* @656 'c' (4 pixels wide) */
	0x1E, 0x00, //    ####    
	0x21, 0x00, //   #    #   
	0x21, 0x00, //   #    #   
	0x21, 0x00, //   #    #   

	/* @664 'd' (5 pixels wide) */
	0x1E, 0x00, //    ####    
	0x21, 0x00, //   #    #   
	0x21, 0x00, //   #    #   
	0x21, 0x00, //   #    #   
	0x3F, 0xE0, //   #########

	/* @674 'e' (5 pixels wide) */
	0x1E, 0x00, //    ####    
	0x25, 0x00, //   #  # #   
	0x25, 0x00, //   #  # #   
	0x25, 0x00, //   #  # #   
	0x16, 0x00, //    # ##    

	/* @684 'f' (3 pixels wide) */
	0x3F, 0xC0, //   ######## 
	0x01, 0x20, //        #  #
	0x01, 0x20, //        #  #

	/* @690 'g' (5 pixels wide) */
	0x1E, 0x00, //    ####    
	0xA1, 0x00, // # #    #   
	0xA1, 0x00, // # #    #   
	0xA1, 0x00, // # #    #   
	0x7F, 0x00, //  #######   

	/* @700 'h' (5 pixels wide) */
	0x3F, 0xE0, //   #########
	0x01, 0x00, //        #   
	0x01, 0x00, //        #   
	0x01, 0x00, //        #   
	0x3E, 0x00, //   #####    

	/* @710 'i' (1 pixels wide) */
	0x3F, 0x40, //   ###### # 

	/* @712 'j' (2 pixels wide) */
	0x81, 0x00, // #      #   
	0x7F, 0x40, //  ####### # 

	/* @716 'k' (5 pixels wide) */
	0x3F, 0xE0, //   #########
	0x04, 0x00, //      #     
	0x0A, 0x00, //     # #    
	0x11, 0x00, //    #   #   
	0x20, 0x00, //   #        

	/* @726 'l' (1 pixels wide) */
	0x3F, 0xE0, //   #########

	/* @728 'm' (7 pixels wide) */
	0x3F, 0x00, //   ######   
	0x01, 0x00, //        #   
	0x01, 0x00, //        #   
	0x3E, 0x00, //   #####    
	0x01, 0x00, //        #   
	0x01, 0x00, //        #   
	0x3E, 0x00, //   #####    

	/* @742 'n' (5 pixels wide) */
	0x3F, 0x00, //   ######   
	0x01, 0x00, //        #   
	0x01, 0x00, //        #   
	0x01, 0x00, //        #   
	0x3E, 0x00, //   #####    

	/* @752 'o' (5 pixels wide) */
	0x1E, 0x00, //    ####    
	0x21, 0x00, //   #    #   
	0x21, 0x00, //   #    #   
	0x21, 0x00, //   #    #   
	0x1E, 0x00, //    ####    

	/* @762 'p' (5 pixels wide) */
	0xFF, 0x00, // ########   
	0x21, 0x00, //   #    #   
	0x21, 0x00, //   #    #   
	0x21, 0x00, //   #    #   
	0x1E, 0x00, //    ####    

	/* @772 'q' (5 pixels wide) */
	0x1E, 0x00, //    ####    
	0x21, 0x00, //   #    #   
	0x21, 0x00, //   #    #   
	0x21, 0x00, //   #    #   
	0xFF, 0x00, // ########   

	/* @782 'r' (3 pixels wide) */
	0x3F, 0x00, //   ######   
	0x02, 0x00, //       #    
	0x01, 0x00, //        #   

	/* @788 's' (4 pixels wide) */
	0x26, 0x00, //   #  ##    
	0x25, 0x00, //   #  # #   
	0x29, 0x00, //   # #  #   
	0x19, 0x00, //    ##  #   

	/* @796 't' (3 pixels wide) */
	0x1F, 0xC0, //    ####### 
	0x21, 0x00, //   #    #   
	0x21, 0x00, //   #    #   

	/* @802 'u' (5 pixels wide) */
	0x1F, 0x00, //    #####   
	0x20, 0x00, //   #        
	0x20, 0x00, //   #        
	0x20, 0x00, //   #        
	0x3F, 0x00, //   ######   

	/* @812 'v' (5 pixels wide) */
	0x03, 0x00, //       ##   
	0x0C, 0x00, //     ##     
	0x30, 0x00, //   ##       
	0x0C, 0x00, //     ##     
	0x03, 0x00, //       ##   

	/* @822 'w' (7 pixels wide) */
	0x0F, 0x00, //     ####   
	0x30, 0x00, //   ##       
	0x0C, 0x00, //     ##     
	0x03, 0x00, //       ##   
	0x0C, 0x00, //     ##     
	0x30, 0x00, //   ##       
	0x0F, 0x00, //     ####   

	/* @836 'x' (5 pixels wide) */
	0x21, 0x00, //   #    #   
	0x12, 0x00, //    #  #    
	0x0C, 0x00, //     ##     
	0x12, 0x00, //    #  #    
	0x21, 0x00, //   #    #   

	/* @846 'y' (5 pixels wide) */
	0x03, 0x00, //       ##   
	0xCC, 0x00, // ##  ##     
	0x30, 0x00, //   ##       
	0x0C, 0x00, //     ##     
	0x03, 0x00, //       ##   

	/* @856 'z' (4 pixels wide) */
	0x31, 0x00, //   ##   #   
	0x29, 0x00, //   # #  #   
	0x25, 0x00, //   #  # #   
	0x23, 0x00, //   #   ##   

	/* @864 '{' (4 pixels wide) */
	0x04, 0x00, //      #     
	0x04, 0x00, //      #     
	0x7B, 0xC0, //  #### #### 
	0x80, 0x20, // #         #

	/* @872 '|' (1 pixels wide) */
	0xFF, 0xE0, // ###########

	/* @874 '}' (4 pixels wide) */
	0x80, 0x20, // #         #
	0x7B, 0xC0, //  #### #### 
	0x04, 0x00, //      #     
	0x04, 0x00, //      #     

	/* @882 '~' (7 pixels wide) */
	0x0C, 0x00, //     ##     
	0x02, 0x00, //       #    
	0x02, 0x00, //       #    
	0x04, 0x00, //      #     
	0x08, 0x00, //     #      
	0x08, 0x00, //     #      
	0x06, 0x00, //      ##    
};

/* Character descriptors for Tahoma 8pt */
/* { [Char width in bits], [Offset into tahoma_8ptCharBitmaps in bytes] } */
const FONT_CHAR_INFO tahoma_8ptDescriptors[] = 
{
	{6, 0}, 		/*   */ 
	{1, 12}, 		/* ! */ 
	{3, 14}, 		/* " */ 
	{7, 20}, 		/* # */ 
	{5, 34}, 		/* $ */ 
	{10, 44}, 		/* % */ 
	{7, 64}, 		/* & */ 
	{1, 78}, 		/* ' */ 
	{3, 80}, 		/* ( */ 
	{3, 86}, 		/* ) */ 
	{5, 92}, 		/* * */ 
	{7, 102}, 		/* + */ 
	{2, 116}, 		/* , */ 
	{3, 120}, 		/* - */ 
	{1, 126}, 		/* . */ 
	{3, 128}, 		/* / */ 
	{5, 134}, 		/* 0 */ 
	{3, 144}, 		/* 1 */ 
	{5, 150}, 		/* 2 */ 
	{5, 160}, 		/* 3 */ 
	{5, 170}, 		/* 4 */ 
	{5, 180}, 		/* 5 */ 
	{5, 190}, 		/* 6 */ 
	{5, 200}, 		/* 7 */ 
	{5, 210}, 		/* 8 */ 
	{5, 220}, 		/* 9 */ 
	{1, 230}, 		/* : */ 
	{2, 232}, 		/* ; */ 
	{6, 236}, 		/* < */ 
	{7, 248}, 		/* = */ 
	{6, 262}, 		/* > */ 
	{4, 274}, 		/* ? */ 
	{9, 282}, 		/* @ */ 
	{6, 300}, 		/* A */ 
	{5, 312}, 		/* B */ 
	{6, 322}, 		/* C */ 
	{6, 334}, 		/* D */ 
	{5, 346}, 		/* E */ 
	{5, 356}, 		/* F */ 
	{6, 366}, 		/* G */ 
	{6, 378}, 		/* H */ 
	{3, 390}, 		/* I */ 
	{4, 396}, 		/* J */ 
	{5, 404}, 		/* K */ 
	{4, 414}, 		/* L */ 
	{7, 422}, 		/* M */ 
	{6, 436}, 		/* N */ 
	{7, 448}, 		/* O */ 
	{5, 462}, 		/* P */ 
	{7, 472}, 		/* Q */ 
	{6, 486}, 		/* R */ 
	{5, 498}, 		/* S */ 
	{5, 508}, 		/* T */ 
	{6, 518}, 		/* U */ 
	{5, 530}, 		/* V */ 
	{9, 540}, 		/* W */ 
	{5, 558}, 		/* X */ 
	{5, 568}, 		/* Y */ 
	{5, 578}, 		/* Z */ 
	{3, 588}, 		/* [ */ 
	{3, 594}, 		/* \ */ 
	{3, 600}, 		/* ] */ 
	{7, 606}, 		/* ^ */ 
	{6, 620}, 		/* _ */ 
	{2, 632}, 		/* ` */ 
	{5, 636}, 		/* a */ 
	{5, 646}, 		/* b */ 
	{4, 656}, 		/* c */ 
	{5, 664}, 		/* d */ 
	{5, 674}, 		/* e */ 
	{3, 684}, 		/* f */ 
	{5, 690}, 		/* g */ 
	{5, 700}, 		/* h */ 
	{1, 710}, 		/* i */ 
	{2, 712}, 		/* j */ 
	{5, 716}, 		/* k */ 
	{1, 726}, 		/* l */ 
	{7, 728}, 		/* m */ 
	{5, 742}, 		/* n */ 
	{5, 752}, 		/* o */ 
	{5, 762}, 		/* p */ 
	{5, 772}, 		/* q */ 
	{3, 782}, 		/* r */ 
	{4, 788}, 		/* s */ 
	{3, 796}, 		/* t */ 
	{5, 802}, 		/* u */ 
	{5, 812}, 		/* v */ 
	{7, 822}, 		/* w */ 
	{5, 836}, 		/* x */ 
	{5, 846}, 		/* y */ 
	{4, 856}, 		/* z */ 
	{4, 864}, 		/* { */ 
	{1, 872}, 		/* | */ 
	{4, 874}, 		/* } */ 
	{7, 882}, 		/* ~ */ 
};

/* Font information for Tahoma 8pt */
const FONT_INFO tahoma_8ptFontInfo =
{
	11, /*  Character height */
    1,
	' ', /*  Start character */
	'~', /*  End character */
	tahoma_8ptDescriptors, /*  Character descriptor array */
	tahoma_8ptBitmaps, /*  Character bitmap array */
    0
};
