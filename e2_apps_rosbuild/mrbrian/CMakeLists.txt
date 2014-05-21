cmake_minimum_required(VERSION 2.4.6)
project(MrBrian)

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#find Bison and Flex packages
find_package(BISON)
find_package(FLEX)

#generate .cpp parsers from flex and Bison sources for libraries
BISON_TARGET(PredParser src/brian/prs/predgram.y ${PROJECT_SOURCE_DIR}/src/brian/src/predgram.tab.cpp COMPILE_FLAGS --defines="${PROJECT_SOURCE_DIR}/src/brian/include/predgram.tab.h" COMPILE_FLAGS "-p pred")
BISON_TARGET(RulesParser src/brian/prs/rulesgram.y ${PROJECT_SOURCE_DIR}/src/brian/src/rulesgram.tab.cpp COMPILE_FLAGS --defines="${PROJECT_SOURCE_DIR}/src/brian/include/rulesgram.tab.h"  COMPILE_FLAGS "-p rules")
BISON_TARGET(AssocParser src/fuzzy/prs/assocgram.y ${PROJECT_SOURCE_DIR}/src/fuzzy/src/assocgram.tab.cpp COMPILE_FLAGS --defines="${PROJECT_SOURCE_DIR}/src/fuzzy/include/assocgram.tab.h"  COMPILE_FLAGS "-p assoc")
BISON_TARGET(ShapeParser src/fuzzy/prs/shapegram.y ${PROJECT_SOURCE_DIR}/src/fuzzy/src/shapegram.tab.cpp COMPILE_FLAGS --defines="${PROJECT_SOURCE_DIR}/src/fuzzy/include/shapegram.tab.h"  COMPILE_FLAGS "-p shape")

FLEX_TARGET(PredScanner src/brian/prs/predflex.l ${PROJECT_SOURCE_DIR}/src/brian/src/predflex.cpp COMPILE_FLAGS "-Ppred")
FLEX_TARGET(RulesScanner src/brian/prs/rulesflex.l  ${PROJECT_SOURCE_DIR}/src/brian/src/rulesflex.cpp COMPILE_FLAGS "-Prules")
FLEX_TARGET(AssocScanner src/fuzzy/prs/assocflex.l ${PROJECT_SOURCE_DIR}/src/fuzzy/src/assocflex.cpp COMPILE_FLAGS "-Passoc")
FLEX_TARGET(ShapeScanner src/fuzzy/prs/shapeflex.l  ${PROJECT_SOURCE_DIR}/src/fuzzy/src/shapeflex.cpp COMPILE_FLAGS "-Pshape")

#generate .cpp parsers from flex and Bison sources for executable
BISON_TARGET(CheckPredParser src/checker/prs/check_predgram.y ${PROJECT_SOURCE_DIR}/src/checker/src/check_predgram.tab.cpp COMPILE_FLAGS --defines="${PROJECT_SOURCE_DIR}/src/checker/include/check_predgram.tab.h" COMPILE_FLAGS "-p pred")
BISON_TARGET(CheckRulesParser src/checker/prs/check_rulesgram.y ${PROJECT_SOURCE_DIR}/src/checker/src/check_rulesgram.tab.cpp COMPILE_FLAGS --defines="${PROJECT_SOURCE_DIR}/src/checker/include/check_rulesgram.tab.h"  COMPILE_FLAGS "-p rules")
BISON_TARGET(CheckAssocParser src/checker/prs/check_assgram.y ${PROJECT_SOURCE_DIR}/src/checker/src/check_assgram.tab.cpp COMPILE_FLAGS --defines="${PROJECT_SOURCE_DIR}/src/checker/include/check_assgram.tab.h"  COMPILE_FLAGS "-p ass")
BISON_TARGET(CheckShapeParser src/checker/prs/check_shapegram.y ${PROJECT_SOURCE_DIR}/src/checker/src/check_shapegram.tab.cpp COMPILE_FLAGS --defines="${PROJECT_SOURCE_DIR}/src/checker/include/check_shapegram.tab.h"  COMPILE_FLAGS "-p shape")

FLEX_TARGET(CheckPredScanner src/checker/prs/check_predflex.l ${PROJECT_SOURCE_DIR}/src/checker/src/check_predflex.cpp COMPILE_FLAGS "-Ppred")
FLEX_TARGET(CheckRulesScanner src/checker/prs/check_rulesflex.l  ${PROJECT_SOURCE_DIR}/src/checker/src/check_rulesflex.cpp COMPILE_FLAGS "-Prules")
FLEX_TARGET(CheckAssocScanner src/checker/prs/check_assflex.l ${PROJECT_SOURCE_DIR}/src/checker/src/check_assflex.cpp COMPILE_FLAGS "-Pass")
FLEX_TARGET(CheckShapeScanner src/checker/prs/check_shapeflex.l  ${PROJECT_SOURCE_DIR}/src/checker/src/check_shapeflex.cpp COMPILE_FLAGS "-Pshape")

#add dependencies between parsers and scanners (.l e .y)
ADD_FLEX_BISON_DEPENDENCY(PredScanner PredParser)
ADD_FLEX_BISON_DEPENDENCY(RulesScanner RulesParser)
ADD_FLEX_BISON_DEPENDENCY(AssocScanner AssocParser)
ADD_FLEX_BISON_DEPENDENCY(ShapeScanner ShapeParser)
ADD_FLEX_BISON_DEPENDENCY(CheckPredScanner CheckPredParser)
ADD_FLEX_BISON_DEPENDENCY(CheckRulesScanner CheckRulesParser)
ADD_FLEX_BISON_DEPENDENCY(CheckAssocScanner CheckAssocParser)
ADD_FLEX_BISON_DEPENDENCY(CheckShapeScanner CheckShapeParser)

FILE(GLOB HEADERS "*.tab.hpp")
#FILE(REMOVE HEADERSTOREMOVE ${HEADERS})

#common commands for building c++ executables and libraries
include_directories( 	${PROJECT_SOURCE_DIR}/src/brian/include
 			${PROJECT_SOURCE_DIR}/src/fuzzy/include
 			${PROJECT_SOURCE_DIR}/src/checker/include
 			${PROJECT_SOURCE_DIR}/src/gmbte/include	
)
add_library(    brian STATIC
		src/brian/src/brian.cpp 
		src/brian/src/behavior_eng.cpp 
		src/brian/src/can_doer.cpp 
		src/brian/src/compose.cpp 	
		src/brian/src/preacher.cpp 
		src/brian/src/predicate.cpp 
		src/brian/src/predicate_list.cpp 
		src/brian/src/predicate_list_debug.cpp
		src/brian/src/proposed_action_list.cpp
		src/brian/src/proposed_action_list_debug.cpp
		src/brian/src/wanter_eng.cpp
		src/brian/src/weight_want.cpp
		src/brian/src/weight_want_list.cpp
		src/brian/src/weight_want_list_debug.cpp
		src/brian/src/engine_objects.cpp 
		src/brian/src/rules_behav.cpp
		${FLEX_PredScanner_OUTPUTS}
		${BISON_PredParser_OUTPUTS}  
	 	${FLEX_RulesScanner_OUTPUTS}
		${BISON_RulesParser_OUTPUTS}
)

add_library(	fuzzy STATIC
		src/fuzzy/src/action.cpp
		src/fuzzy/src/command.cpp
		src/fuzzy/src/div_triangle.cpp
		src/fuzzy/src/getFuzzy.cpp
		src/fuzzy/src/shape_point.cpp
		src/fuzzy/src/action_list.cpp
		src/fuzzy/src/command_list.cpp
		src/fuzzy/src/fuzzy_crisp_rel.cpp
		src/fuzzy/src/point.cpp
		src/fuzzy/src/shapes_list.cpp
		src/fuzzy/src/action_list_debug.cpp
		src/fuzzy/src/command_list_debug.cpp
		src/fuzzy/src/fuzzy_data.cpp
		src/fuzzy/src/point_list.cpp
		src/fuzzy/src/singleton.cpp
		src/fuzzy/src/assoc_file_parser.cpp
		src/fuzzy/src/crisp_data.cpp
		src/fuzzy/src/fuzzy_data_list.cpp
		src/fuzzy/src/point_multimap.cpp
		src/fuzzy/src/singleton_defuzzyfier.cpp
		src/fuzzy/src/association.cpp
		src/fuzzy/src/crisp_data_list.cpp
		src/fuzzy/src/fuzzy_data_list_debug.cpp
		src/fuzzy/src/rectangle.cpp
		src/fuzzy/src/trapezium.cpp
		src/fuzzy/src/association_list.cpp
		src/fuzzy/src/crisp_data_list_debug.cpp
		src/fuzzy/src/fuzzyfier.cpp
		src/fuzzy/src/shape.cpp
		src/fuzzy/src/triangle.cpp
		src/fuzzy/src/association_set_multimap.cpp
		src/fuzzy/src/data.cpp
		src/fuzzy/src/fuzzy_set.cpp
		src/fuzzy/src/shape_file_parser.cpp
		src/fuzzy/src/triangle_ol.cpp
		src/fuzzy/src/barycentre_defuzzyfier.cpp
		src/fuzzy/src/defuzzyfier.cpp
		src/fuzzy/src/fuzzy_set_multimap.cpp
		src/fuzzy/src/shape_multimap.cpp
		src/fuzzy/src/triangle_or.cpp
		${FLEX_AssocScanner_OUTPUTS}
		${BISON_AssocParser_OUTPUTS}  
	 	${FLEX_ShapeScanner_OUTPUTS}
		${BISON_ShapeParser_OUTPUTS}
)

link_directories(${PROJECT_SOURCE_DIR}/lib)
add_executable(	checker	
		src/checker/src/check.cpp
		${FLEX_CheckPredScanner_OUTPUTS}
		${BISON_CheckPredParser_OUTPUTS}  
	 	${FLEX_CheckRulesScanner_OUTPUTS}
		${BISON_CheckRulesParser_OUTPUTS}
		${FLEX_CheckAssocScanner_OUTPUTS}
		${BISON_CheckAssocParser_OUTPUTS}  
	 	${FLEX_CheckShapeScanner_OUTPUTS}
		${BISON_CheckShapeParser_OUTPUTS}
)
add_dependencies(checker brian fuzzy)
target_link_libraries(checker libfuzzy.a libbrian.a)

find_package(GTK2)
find_package(ATK)

include_directories (${GTK2_INCLUDE_DIRS} ${ATK_INCLUDE_DIR})
link_directories (${GTK2_LIBRARY_DIRS})
add_definitions (${GTK2_CFLAGS_OTHER})
add_executable( gmbte
		src/gmbte/src/callbacks.cpp  
		src/gmbte/src/interface.cpp
		src/gmbte/src/main.cpp
		src/gmbte/src/support.cpp
)
add_dependencies(gmbte brian fuzzy)
target_link_libraries(gmbte libbrian.a libfuzzy.a ${GTK2_LIBRARIES} ${ATK_LIBRARIES})

set (CMAKE_INSTALL_PREFIX  /usr/local/lib/airlab/mrbrian)
#install (TARGETS gmbte checker DESTINATION bin )
install (TARGETS gmbte checker brian fuzzy
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib)

FILE(GLOB MyBrianHeaders "${PROJECT_SOURCE_DIR}/src/brian/include/*.h")
install (FILES ${MyBrianHeaders} DESTINATION include/brian)
install (FILES ${MyBrianHeaders} DESTINATION ${PROJECT_SOURCE_DIR}/include/brian)

FILE(GLOB MyFuzzyHeaders "${PROJECT_SOURCE_DIR}/src/fuzzy/include/*.h")
install (FILES ${MyFuzzyHeaders} DESTINATION include/fuzzy)
install (FILES ${MyFuzzyHeaders} DESTINATION ${PROJECT_SOURCE_DIR}/include/fuzzy)

install (FILES ${PROJECT_SOURCE_DIR}/src/gmbte/icons/brainicon.png DESTINATION bin)
install (FILES ${PROJECT_SOURCE_DIR}/src/gmbte/icons/brainicon.png DESTINATION ${PROJECT_SOURCE_DIR}/bin)


