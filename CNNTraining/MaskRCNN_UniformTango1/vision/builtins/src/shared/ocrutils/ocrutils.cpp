/* Copyright 2013-2024 The MathWorks, Inc. */
////////////////////////////////////////////////////////////////////////////////
// This file provides shared utilties used in both the ocr built-in function
// and the codegen interface. 
////////////////////////////////////////////////////////////////////////////////

#include <ocrutils/ocrutils_util.hpp>
#include <ocrutils/ocrutils_published_c_api.hpp>
#include <ocrutils/pix_utils.hpp>
#include <ocrutils/MetadataCollection.hpp>
#include <ocrutils/metadata_utils.hpp>

#ifdef COMPILE_FOR_VISION_BUILTINS
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

#include "tesseract/baseapi.h"
#include "leptonica/allheaders.h"


////////////////////////////////////////////////////////////////////////////////
// Error codes used to report errors in codegen. 
// INIT_ERROR     - failed to initialize tesseract
// PIXALLOC_ERROR - failed to allocate memory for Pix image
// UNKNOWN_ERROR  - unknown error
//////////////////////////////////////////////////////////////////////////////////
#define INIT_ERROR     -1
#define PIXALLOC_ERROR -2
#define UNKNOWN_ERROR  -3

////////////////////////////////////////////////////////////////////////////////
// Set namespace to ocrutils
////////////////////////////////////////////////////////////////////////////////
using namespace ocrutils;

///////////////////////////////////////////////////////////////////////////////
// Cleanup tesseract 
///////////////////////////////////////////////////////////////////////////////
void cleanupTesseract(void * tessAPI)
{
    // shutdown tesseract.
    ((tesseract::TessBaseAPI *)tessAPI)->End();      

    delete (tesseract::TessBaseAPI *)tessAPI;
}

////////////////////////////////////////////////////////////////////////////////
// Copy recognized text into output array and delete the source
////////////////////////////////////////////////////////////////////////////////
void copyTextAndCleanup(char * src, uint8_T * dest, const size_t length)
{
    // dest is uint8 to match the uint8 buffer allocated by MATLAB Coder. We
    // need this because char can be signed or unsigned depending on the
    // platform implementation. This can affect the byte parsing code we use
    // to parse multi-byte strings. 

    // copy text data created by tesseract into allocated output buffer.
    strncpy((char *)dest,src,length);

    // delete the text data created by tesseract.
    delete[] src;
}

////////////////////////////////////////////////////////////////////////////////
// Free memory used by MetadataCollection
////////////////////////////////////////////////////////////////////////////////
void cleanupMetadata(void * ocrMetadata)
{
    delete (MetadataCollection *)ocrMetadata;
}

////////////////////////////////////////////////////////////////////////////////
// Collect OCR metadata from tesseract and determine the buffer sizes we need
// to hold the metadata. The buffers are filled in the copyMetadataCollection function.
////////////////////////////////////////////////////////////////////////////////
void collectMetadata(void * tessAPI, void ** ocrMetadata, int32_T * numChars,
        int32_T * numWords, int32_T * numTextLines,
        int32_T * numParagraphs, int32_T * numBlocks)
{

    MetadataCollection * metadata = new MetadataCollection;
    *ocrMetadata = metadata;

    tesseract::TessBaseAPI * tess = (tesseract::TessBaseAPI *)tessAPI;

    // get iterator to tesseract's results
    tesseract::ResultIterator * resultIterator = tess->GetIterator();

    metadata->collectMetadata(resultIterator);
    metadata->assignSizes(numChars, numWords, numTextLines, numParagraphs, numBlocks);
    
    delete resultIterator;

}

////////////////////////////////////////////////////////////////////////////////
// Copy OCR metadata into output buffers.
////////////////////////////////////////////////////////////////////////////////
void copyMetadata(void * ocrMetadata,
        double * charBBox,
        int32_T * charWordIndex,
        float * charConfidence,
        double * wordBBox,
        int32_T * wordTextLineIndex,
        float * wordConfidence,
        int32_T * wordCharacterIndex,
        double * textlineBBox,
        int32_T * textlineParagraphIndex,
        float * textlineConfidence,
        int32_T * textlineCharacterIndex,
        double * paragraphBBox,
        int32_T * paragraphBlockIndex,
        float * paragraphConfidence,
        int32_T * paragraphCharacterIndex,
        double * blockBBox,
        int32_T * blockPageIndex,
        float * blockConfidence,
        int32_T * blockCharacterIndex)
{

    MetadataCollection * metadata = (MetadataCollection *)ocrMetadata;
    copyMetadata(metadata->mCharacters, charBBox, charWordIndex,
            charConfidence,NULL,true);

    copyMetadata(metadata->mWords, wordBBox, wordTextLineIndex,
            wordConfidence, wordCharacterIndex);

    copyMetadata(metadata->mTextLines,textlineBBox,textlineParagraphIndex,
            textlineConfidence,textlineCharacterIndex);

    copyMetadata(metadata->mParagraphs, paragraphBBox, paragraphBlockIndex,
            paragraphConfidence, paragraphCharacterIndex);

    copyMetadata(metadata->mBlocks, blockBBox, blockPageIndex,
            blockConfidence, blockCharacterIndex);
}

///////////////////////////////////////////////////////////////////////////////
// Get text from metadata and copy into output, utf8Text. And return number of
// characters.
///////////////////////////////////////////////////////////////////////////////
int32_T getTextFromMetadata(void * ocrMetadata, char ** utf8Text)
{

    MetadataCollection *metadata = (MetadataCollection *)ocrMetadata;

    auto utf8String = getTextFromMetadata(metadata->mCharacters);

    // allocate memory and copy text to output. 
    auto cstr  = utf8String.c_str();

    *utf8Text  = new char[strlen(cstr) + 1]; // include space for null terminator

    strcpy(*utf8Text, cstr);

    return static_cast<int32_T>(strlen(*utf8Text)); // return size of text buffer

}

int32_T tesseractRecognizeText(void ** tessAPI,
        const void * I, char ** utf8Text,
        const int32_T width, const int32_T height,
        const char * textLayout, const char * charSet,
        const char * tessdata, const char * lang,
        const uint8_T depth, const boolean_T resetParams,
        const uint8_T engine)
{

    // These vectors are for tesseract's parameters
    std::vector<std::string> paramNames;
    std::vector<std::string> paramValues;

    // TextLayout parameter
    paramNames.push_back("tessedit_pageseg_mode");
    paramValues.push_back(textLayout);

    // CharacterSet parameter
    paramNames.push_back("tessedit_char_whitelist");
    paramValues.push_back(charSet);

    tesseract::TessBaseAPI * tess = new tesseract::TessBaseAPI;
    *tessAPI = tess;

    if (resetParams)
    {
        resetGlobalParameters(tess);
    }

    auto oem = static_cast<tesseract::OcrEngineMode>(engine);
    int err = INIT_ERROR;
    try 
    {
        // try to init tesseract given the tessdata location.
        err = tess->Init(tessdata, lang, oem, NULL, 0, &paramNames, &paramValues, false);

        // if first attempt fails, check for tessdata folder in current 
        // directory. This allows deployment as long as tessdata folder
        // is stored in same location as executable.
        if (err < 0)
        {
            err = tess->Init(".", lang, oem, NULL, 0, &paramNames, &paramValues, false);
        }

        // Normal control flow.
        if (err < 0)
        {
            return INIT_ERROR; 
        }
        else
        {
            // convert image to Pix format
            Pix * pix = pixCreateNoInit(width, height, depth /*uint8 */);

            if (pix == NULL)
            {
                return PIXALLOC_ERROR; 
            }
            else
            {
                if (depth==1)
                {
                    copyMxLogicalsToPix((const boolean_T*)I, pix, width, height);
                }
                else
                {
                    copyMxUint8DataToPix((const uint8_T *)I, pix, width, height);
                }

                tess->SetImage(pix);

                // recognized text is returned in null terminated C-style char array
                *utf8Text = tess->GetUTF8Text(); // returns results

                // clean-up
                pixDestroy(&pix);
                return static_cast<int32_T>(strlen(*utf8Text)); // return size of text buffer
            }
        }
        return UNKNOWN_ERROR; 
    }
    catch (...)    
    {
        // Tesseract threw an internal error.
        return UNKNOWN_ERROR;
    }


}


////////////////////////////////////////////////////////////////////////////////
// Recognize text in image using tesseract. This function is for uint8 images.
////////////////////////////////////////////////////////////////////////////////
int32_T tesseractRecognizeTextUint8(void ** tessAPI,
        const uint8_T * I, char ** utf8Text,
        const int32_T width, const int32_T height,
        const char * textLayout, const char * charSet,
        const char * tessdata, const char * lang, 
        const boolean_T resetParams, const uint8_T engine)
{

    return tesseractRecognizeText(tessAPI, (const void *)I, utf8Text, width,
            height, textLayout, charSet, tessdata,
            lang, 8 /* depth */, resetParams, engine);

}

////////////////////////////////////////////////////////////////////////////////
// Recognize text in image. This function is for binary images. 
////////////////////////////////////////////////////////////////////////////////
int32_T tesseractRecognizeTextLogical(void ** tessAPI,
        const boolean_T * I, char ** utf8Text,
        const int32_T width, const int32_T height,
        const char * textLayout, const char * charSet,
        const char * tessdata, const char * lang, 
        const boolean_T resetParams, const uint8_T engine)
{

    return tesseractRecognizeText(tessAPI, (const void *)I, utf8Text, width,
            height, textLayout, charSet, tessdata,
            lang, 1 /* depth */, resetParams, engine);

}

///////////////////////////////////////////////////////////////////////////////
// Reset all of tesseracts global parameters. 
//
// Tesseract v3.02 does not reset global parameters on language change forcing
// us to reset manually.
///////////////////////////////////////////////////////////////////////////////
void resetGlobalParameters(tesseract::TessBaseAPI * tess)
{
    tess->SetVariable("textord_show_fixed_cuts", "0");
    tess->SetVariable("devanagari_split_debugimage", "0");
    tess->SetVariable("textord_really_old_xheight", "0");
    tess->SetVariable("textord_oldbl_debug", "0");
    tess->SetVariable("textord_debug_baselines", "0");
    tess->SetVariable("textord_oldbl_paradef", "1");
    tess->SetVariable("textord_oldbl_split_splines", "1");
    tess->SetVariable("textord_oldbl_merge_parts", "1");
    tess->SetVariable("oldbl_corrfix", "1");
    tess->SetVariable("oldbl_xhfix", "0");
    tess->SetVariable("textord_ocropus_mode", "0");
    tess->SetVariable("textord_all_prop", "0");
    tess->SetVariable("textord_debug_pitch_test", "0");
    tess->SetVariable("textord_disable_pitch_test", "0");
    tess->SetVariable("textord_fast_pitch_test", "0");
    tess->SetVariable("textord_debug_pitch_metric", "0");
    tess->SetVariable("textord_show_row_cuts", "0");
    tess->SetVariable("textord_show_page_cuts", "0");
    tess->SetVariable("textord_pitch_cheat", "0");
    tess->SetVariable("textord_blockndoc_fixed", "0");
    tess->SetVariable("edges_use_new_outline_complexity", "0");
    tess->SetVariable("edges_debug", "0");
    tess->SetVariable("edges_children_fix", "0");
    tess->SetVariable("textord_fp_chopping", "1");
    tess->SetVariable("textord_force_make_prop_words", "0");
    tess->SetVariable("textord_chopper_test", "0");
    tess->SetVariable("textord_restore_underlines", "1");
    tess->SetVariable("textord_dump_table_images", "0");
    tess->SetVariable("textord_show_tables", "0");
    tess->SetVariable("textord_tablefind_show_mark", "0");
    tess->SetVariable("textord_tablefind_show_stats", "0");
    tess->SetVariable("textord_tablefind_recognize_tables", "0");
    tess->SetVariable("textord_tabfind_only_strokewidths", "0");
    tess->SetVariable("textord_tabfind_vertical_text", "1");
    tess->SetVariable("textord_tabfind_force_vertical_text", "0");
    tess->SetVariable("textord_tabfind_vertical_horizontal_mix", "1");
    tess->SetVariable("textord_debug_images", "0");
    tess->SetVariable("textord_debug_printable", "0");
    tess->SetVariable("textord_space_size_is_variable", "0");
    tess->SetVariable("textord_show_initial_words", "0");
    tess->SetVariable("textord_show_new_words", "0");
    tess->SetVariable("textord_show_fixed_words", "0");
    tess->SetVariable("textord_blocksall_fixed", "0");
    tess->SetVariable("textord_blocksall_prop", "0");
    tess->SetVariable("textord_blocksall_testing", "0");
    tess->SetVariable("textord_test_mode", "0");
    tess->SetVariable("textord_pitch_scalebigwords", "0");
    tess->SetVariable("textord_heavy_nr", "0");
    tess->SetVariable("textord_show_initial_rows", "0");
    tess->SetVariable("textord_show_parallel_rows", "0");
    tess->SetVariable("textord_show_expanded_rows", "0");
    tess->SetVariable("textord_show_final_rows", "0");
    tess->SetVariable("textord_show_final_blobs", "0");
    tess->SetVariable("textord_test_landscape", "0");
    tess->SetVariable("textord_parallel_baselines", "1");
    tess->SetVariable("textord_straight_baselines", "0");
    tess->SetVariable("textord_old_baselines", "1");
    tess->SetVariable("textord_old_xheight", "0");
    tess->SetVariable("textord_fix_xheight_bug", "1");
    tess->SetVariable("textord_fix_makerow_bug", "1");
    tess->SetVariable("textord_debug_xheights", "0");
    tess->SetVariable("textord_biased_skewcalc", "1");
    tess->SetVariable("textord_interpolating_skew", "1");
    tess->SetVariable("textord_new_initial_xheight", "1");
    tess->SetVariable("textord_tabfind_show_initialtabs", "0");
    tess->SetVariable("textord_tabfind_show_finaltabs", "0");
    tess->SetVariable("textord_tabfind_show_color_fit", "0");
    tess->SetVariable("gapmap_debug", "0");
    tess->SetVariable("gapmap_use_ends", "0");
    tess->SetVariable("gapmap_no_isolated_quanta", "0");
    tess->SetVariable("textord_tabfind_show_initial_partitions", "0");
    tess->SetVariable("textord_tabfind_show_reject_blobs", "0");
    tess->SetVariable("textord_tabfind_show_columns", "0");
    tess->SetVariable("textord_tabfind_show_blocks", "0");
    tess->SetVariable("textord_tabfind_find_tables", "1");
    tess->SetVariable("wordrec_display_all_blobs", "0");
    tess->SetVariable("wordrec_display_all_words", "0");
    tess->SetVariable("wordrec_blob_pause", "0");
    tess->SetVariable("equationdetect_save_bi_image", "0");
    tess->SetVariable("equationdetect_save_spt_image", "0");
    tess->SetVariable("equationdetect_save_seed_image", "0");
    tess->SetVariable("equationdetect_save_merged_image", "0");
    tess->SetVariable("wordrec_display_splits", "0");
    tess->SetVariable("poly_debug", "0");
    tess->SetVariable("poly_wide_objects_better", "1");
    tess->SetVariable("oldbl_xhfract", "0.4");
    tess->SetVariable("oldbl_dot_error_size", "1.26");
    tess->SetVariable("textord_oldbl_jumplimit", "0.15");
    tess->SetVariable("textord_projection_scale", "0.200");
    tess->SetVariable("textord_balance_factor", "1.0");
    tess->SetVariable("pitsync_joined_edge", "0.75");
    tess->SetVariable("pitsync_offset_freecut_fraction", "0.25");
    tess->SetVariable("textord_underline_threshold", "0.5");
    tess->SetVariable("edges_childarea", "0.5");
    tess->SetVariable("edges_boxarea", "0.875");
    tess->SetVariable("textord_fp_chop_snap", "0.5");
    tess->SetVariable("textord_underline_offset", "0.1");
    tess->SetVariable("textord_tabfind_vertical_text_ratio", "0.5");
    tess->SetVariable("textord_wordstats_smooth_factor", "0.05");
    tess->SetVariable("textord_width_smooth_factor", "0.10");
    tess->SetVariable("textord_words_width_ile", "0.4");
    tess->SetVariable("textord_words_maxspace", "4.0");
    tess->SetVariable("textord_words_default_maxspace", "3.5");
    tess->SetVariable("textord_words_default_minspace", "0.6");
    tess->SetVariable("textord_words_min_minspace", "0.3");
    tess->SetVariable("textord_words_default_nonspace", "0.2");
    tess->SetVariable("textord_words_initial_lower", "0.25");
    tess->SetVariable("textord_words_initial_upper", "0.15");
    tess->SetVariable("textord_words_minlarge", "0.75");
    tess->SetVariable("textord_words_pitchsd_threshold", "0.040");
    tess->SetVariable("textord_words_def_fixed", "0.016");
    tess->SetVariable("textord_words_def_prop", "0.090");
    tess->SetVariable("textord_pitch_rowsimilarity", "0.08");
    tess->SetVariable("words_initial_lower", "0.5");
    tess->SetVariable("words_initial_upper", "0.15");
    tess->SetVariable("words_default_prop_nonspace", "0.25");
    tess->SetVariable("words_default_fixed_space", "0.75");
    tess->SetVariable("words_default_fixed_limit", "0.6");
    tess->SetVariable("textord_words_definite_spread", "0.30");
    tess->SetVariable("textord_spacesize_ratiofp", "2.8");
    tess->SetVariable("textord_spacesize_ratioprop", "2.0");
    tess->SetVariable("textord_fpiqr_ratio", "1.5");
    tess->SetVariable("textord_max_pitch_iqr", "0.20");
    tess->SetVariable("textord_fp_min_width", "0.5");
    tess->SetVariable("textord_spline_shift_fraction", "0.02");
    tess->SetVariable("textord_spline_outlier_fraction", "0.1");
    tess->SetVariable("textord_skew_ile", "0.5");
    tess->SetVariable("textord_skew_lag", "0.01");
    tess->SetVariable("textord_linespace_iqrlimit", "0.2");
    tess->SetVariable("textord_width_limit", "8");
    tess->SetVariable("textord_chop_width", "1.5");
    tess->SetVariable("textord_expansion_factor", "1.0");
    tess->SetVariable("textord_overlap_x", "0.5");
    tess->SetVariable("textord_minxh", "0.25");
    tess->SetVariable("textord_min_linesize", "1.25");
    tess->SetVariable("textord_excess_blobsize", "1.3");
    tess->SetVariable("textord_occupancy_threshold", "0.4");
    tess->SetVariable("textord_underline_width", "2.0");
    tess->SetVariable("textord_min_blob_height_fraction", "0.75");
    tess->SetVariable("textord_xheight_mode_fraction", "0.4");
    tess->SetVariable("textord_ascheight_mode_fraction", "0.08");
    tess->SetVariable("textord_descheight_mode_fraction", "0.08");
    tess->SetVariable("textord_ascx_ratio_min", "1.25");
    tess->SetVariable("textord_ascx_ratio_max", "1.8");
    tess->SetVariable("textord_descx_ratio_min", "0.25");
    tess->SetVariable("textord_descx_ratio_max", "0.6");
    tess->SetVariable("textord_xheight_error_margin", "0.1");
    tess->SetVariable("textord_tabvector_vertical_gap_fraction", "0.5");
    tess->SetVariable("textord_tabvector_vertical_box_ratio", "0.5");
    tess->SetVariable("textord_tabfind_aligned_gap_fraction", "0.75");
    tess->SetVariable("gapmap_big_gaps", "1.75");
    tess->SetVariable("classify_pico_feature_length", "0.05");
    tess->SetVariable("classify_min_slope", "0.414213562");
    tess->SetVariable("classify_max_slope", "2.414213562");
    tess->SetVariable("classify_cp_angle_pad_loose", "45.0");
    tess->SetVariable("classify_cp_angle_pad_medium", "20.0");
    tess->SetVariable("classify_cp_angle_pad_tight", "10.0");
    tess->SetVariable("classify_cp_end_pad_loose", "0.5");
    tess->SetVariable("classify_cp_end_pad_medium", "0.5");
    tess->SetVariable("classify_cp_end_pad_tight", "0.5");
    tess->SetVariable("classify_cp_side_pad_loose", "2.5");
    tess->SetVariable("classify_cp_side_pad_medium", "1.2");
    tess->SetVariable("classify_cp_side_pad_tight", "0.6");
    tess->SetVariable("classify_pp_angle_pad", "45.0");
    tess->SetVariable("classify_pp_end_pad", "0.5");
    tess->SetVariable("classify_pp_side_pad", "2.5");
    tess->SetVariable("classify_norm_adj_midpoint", "32.0");
    tess->SetVariable("classify_norm_adj_curl", "2.0");
    tess->SetVariable("speckle_large_max_size", "0.30");
    tess->SetVariable("speckle_small_penalty", "10.0");
    tess->SetVariable("speckle_large_penalty", "10.0");
    tess->SetVariable("speckle_small_certainty", "-1.0");
    tess->SetVariable("training_angle_match_scale", "1.0");
    tess->SetVariable("training_similarity_midpoint", "0.0075");
    tess->SetVariable("training_similarity_curl", "2.0");
    tess->SetVariable("training_tangent_bbox_pad", "0.5");
    tess->SetVariable("training_orthogonal_bbox_pad", "2.5");
    tess->SetVariable("training_angle_pad", "45.0");
    tess->SetVariable("fx_debugfile", "DEBUG_WIN_NAME");
    tess->SetVariable("editor_image_win_name", "EditorImage");
    tess->SetVariable("editor_dbwin_name", "EditorDBWin");
    tess->SetVariable("editor_word_name", "BlnWords");
    tess->SetVariable("editor_debug_config_file", "");
    tess->SetVariable("classify_font_name", "UnknownFont");
    tess->SetVariable("classify_training_file", "MicroFeatures");
    tess->SetVariable("devanagari_split_debuglevel", "0");
    tess->SetVariable("oldbl_holed_losscount", "10");
    tess->SetVariable("pitsync_linear_version", "6");
    tess->SetVariable("pitsync_fake_depth", "1");
    tess->SetVariable("edges_max_children_per_outline", "10");
    tess->SetVariable("edges_max_children_layers", "5");
    tess->SetVariable("edges_children_per_grandchild", "10");
    tess->SetVariable("edges_children_count_limit", "45");
    tess->SetVariable("edges_min_nonhole", "12");
    tess->SetVariable("edges_patharea_ratio", "40");
    tess->SetVariable("textord_fp_chop_error", "2");
    tess->SetVariable("textord_tabfind_show_strokewidths", "0");
    tess->SetVariable("textord_debug_tabfind", "0");
    tess->SetVariable("textord_debug_bugs", "0");
    tess->SetVariable("textord_testregion_left", "-1");
    tess->SetVariable("textord_testregion_top", "-1");
    tess->SetVariable("textord_testregion_right", "MAX_INT32");
    tess->SetVariable("textord_testregion_bottom", "MAX_INT32");
    tess->SetVariable("textord_dotmatrix_gap", "3");
    tess->SetVariable("textord_debug_block", "0");
    tess->SetVariable("textord_pitch_range", "2");
    tess->SetVariable("textord_words_veto_power", "5");
    tess->SetVariable("textord_tabfind_show_images", "0");
    tess->SetVariable("textord_skewsmooth_offset", "2");
    tess->SetVariable("textord_skewsmooth_offset2", "1");
    tess->SetVariable("textord_test_x", "-1");
    tess->SetVariable("textord_test_y", "-1");
    tess->SetVariable("textord_min_blobs_in_row", "4");
    tess->SetVariable("textord_spline_minblobs", "8");
    tess->SetVariable("textord_spline_medianwin", "6");
    tess->SetVariable("textord_max_blob_overlaps", "4");
    tess->SetVariable("textord_min_xheight", "10");
    tess->SetVariable("textord_lms_line_trials", "12");
    tess->SetVariable("textord_tabfind_show_partitions", "0");
    tess->SetVariable("edges_maxedgelength", "16000");
    tess->SetVariable("wordrec_display_segmentations", "0");
    tess->SetVariable("editor_image_xpos", "590");
    tess->SetVariable("editor_image_ypos", "10");
    tess->SetVariable("editor_image_menuheight", "50");
    tess->SetVariable("editor_image_word_bb_color", "ScrollView::BLUE");
    tess->SetVariable("editor_image_blob_bb_color", "ScrollView::YELLOW");
    tess->SetVariable("editor_image_text_color", "ScrollView::WHITE");
    tess->SetVariable("editor_dbwin_xpos", "50");
    tess->SetVariable("editor_dbwin_ypos", "500");
    tess->SetVariable("editor_dbwin_height", "24");
    tess->SetVariable("editor_dbwin_width", "80");
    tess->SetVariable("editor_word_xpos", "60");
    tess->SetVariable("editor_word_ypos", "510");
    tess->SetVariable("editor_word_height", "240");
    tess->SetVariable("editor_word_width", "655");
    tess->SetVariable("classify_radius_gyr_min_man", "255");
    tess->SetVariable("classify_radius_gyr_min_exp", "0");
    tess->SetVariable("classify_radius_gyr_max_man", "158");
    tess->SetVariable("classify_radius_gyr_max_exp", "8");
    tess->SetVariable("classify_num_cp_levels", "3");
    tess->SetVariable("image_default_resolution", "300");
}
