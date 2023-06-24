'''
Created on 13.03.2023

@author: Andy Chien
'''
from flexbe_core import EventState, Logger

class GetPaintingParams(EventState):
    '''
    Set/create userdata value from input userdata. (userdata_src_names and userdata_dst_names must be same size.)

    -- painting_area 	   double list[n][2][3]	 A list containing position of two corner.
    -- img_path            string list[m]
    -- area_index_of_image int list[m]

    '''

    def __init__(self, painting_areas, image_paths, area_index_of_image):
        '''
        Constructor
        '''
        super(GetPaintingParams, self).__init__(outcomes=['done', 'finish'],
                                                      output_keys=['painting_area', 'image_path'])
        self._logger = GetPaintingParams._node.get_logger()
        self._painting_areas = painting_areas
        self._image_paths = image_paths
        self._area_index_of_image = area_index_of_image
        self._img_indx = 0

    def execute(self, userdata):
        if self._img_indx >= len(self._image_paths):
            return 'finish'
        userdata.image_path = self._image_paths[self._img_indx]
        userdata.painting_area = self._painting_areas[self._area_index_of_image[self._img_indx]]
        self._img_indx += 1
        return 'done'