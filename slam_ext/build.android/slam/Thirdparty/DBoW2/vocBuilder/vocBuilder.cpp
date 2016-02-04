/**
 * File: Demo.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: demo application of DBoW2
 * License: see the LICENSE.txt file
 */

#include <iostream>
#include <vector>

// DBoW2
#include <../DBoW2/FORB.h>
#include <../DBoW2/TemplatedVocabulary.h>
#include <../DBoW2/ScoringObject.h>

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <iostream>


// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
//#include <opencv/imgproc.h>
#if CV24
#include <opencv2/nonfree/features2d.hpp>
#endif


using namespace DBoW2;
using namespace std;


typedef DBoW2::TemplatedVocabulary<FORB::TDescriptor, FORB>
  ORBVocabulary;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void loadFeatures(vector<vector<FORB::TDescriptor > > &features);
void changeStructure(const cv::Mat &plain, vector<FORB::TDescriptor > &out,
  int L);
void testVocCreation(const vector<vector<FORB::TDescriptor > > &features);
int getdir (string dir, vector<string> &files);
//void testDatabase(const vector<vector<FORB::TDescriptor > > &features);


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// number of training images
//const int NIMAGES = 75;

// extended surf gives 128-dimensional vectors
const bool EXTENDED_SURF = false;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void wait()
{
  cout << endl << "Press enter to continue" << endl;
  getchar();
}

// ----------------------------------------------------------------------------

int main()
{
  vector<vector<FORB::TDescriptor > > features;
  loadFeatures(features);

  testVocCreation(features);

  return 0;
}

/*function... might want it in some class?*/
int getdir (string dir, vector<string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
        files.push_back(string(dirp->d_name));
    }
    closedir(dp);
    return 0;
}

// ----------------------------------------------------------------------------
int NIMAGES=0;
void loadFeatures(vector<vector<FORB::TDescriptor > > &features)
{
    string dir = string("images");
    vector<string> files = vector<string>();

    getdir(dir,files);

    NIMAGES = files.size();

  features.clear();
  features.reserve(NIMAGES);

#ifdef USE_FAST_FEATURES

    //cv::ORB extractor;
    cv::BriefDescriptorExtractor extractor;

  cout << "Extracting FAST features..." << endl;
#else
  cv::ORB extractor;

  cout << "Extracting ORB features..." << endl;
#endif

  for(int i = 0; i < NIMAGES; ++i)
  {
    stringstream ss;
    ss << "images/" << files[i];

    cv::Mat image = cv::imread(ss.str(), 0);
    if (image.channels() > 1){
      cvtColor(image, image, CV_BGR2GRAY);
    }
    cv::Mat mask;
    vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

#ifdef USE_FAST_FEATURES
      cv::FAST(image, keypoints, 20);

        extractor.compute(image, keypoints, descriptors);

#else
      extractor(image, mask, keypoints, descriptors);
#endif

    features.push_back(vector<FORB::TDescriptor >());
    changeStructure(descriptors, features.back(), extractor.descriptorSize());
  }
}

// ----------------------------------------------------------------------------

void changeStructure(const cv::Mat &plain, vector<FORB::TDescriptor > &out,
  int L)
{
  out.resize(plain.rows);
    //unsigned char one = 1;
    for(unsigned int i = 0; i < plain.rows; i++)
    {
      out[i] = FORB::TDescriptor(plain, cv::Rect(0, i, L, 1));//    1, L, CV_8UC1, cv::Scalar(0));
//      const unsigned char* row = plain.ptr<unsigned char>(i);
//      for (unsigned int j = 0; j < plain.cols; ++j){
//        out[i][j] = static_cast<float>(row[j]);
//      }
    }
}

// ----------------------------------------------------------------------------

void testVocCreation(const vector<vector<FORB::TDescriptor > > &features)
{
  // branching factor and depth levels
  const int k = 10;
  const int L = 6;
  const WeightingType weight = TF_IDF;
  const ScoringType score = L1_NORM;

  ORBVocabulary voc(k, L, weight, score);

  cout << "Creating a small " << k << "^" << L << " vocabulary..." << endl;
  voc.create(features);
  cout << "... done!" << endl;

  cout << "Vocabulary information: " << endl
  << voc << endl << endl;

  // lets do something with this vocabulary
//  cout << "Matching images against themselves (0 low, 1 high): " << endl;
//  BowVector v1, v2;
//  for(int i = 0; i < NIMAGES; i++)
//  {
//    voc.transform(features[i], v1);
//    for(int j = 0; j < NIMAGES; j++)
//    {
//      voc.transform(features[j], v2);
//
//      double score = voc.score(v1, v2);
//      cout << "Image " << i << " vs Image " << j << ": " << score << endl;
//    }
//  }

  // save the vocabulary to disk
  cout << endl << "Saving vocabulary..." << endl;
  voc.save("small_voc.yml.gz");
  cout << "Done" << endl;
}

// ----------------------------------------------------------------------------

//void testDatabase(const vector<vector<vector<float> > > &features)
//{
//  cout << "Creating a small database..." << endl;
//
//  // load the vocabulary from disk
//  Surf64Vocabulary voc("small_voc.yml.gz");
//
//  Surf64Database db(voc, false, 0); // false = do not use direct index
//  // (so ignore the last param)
//  // The direct index is useful if we want to retrieve the features that
//  // belong to some vocabulary node.
//  // db creates a copy of the vocabulary, we may get rid of "voc" now
//
//  // add images to the database
//  for(int i = 0; i < NIMAGES; i++)
//  {
//    db.add(features[i]);
//  }
//
//  cout << "... done!" << endl;
//
//  cout << "Database information: " << endl << db << endl;
//
//  // and query the database
//  cout << "Querying the database: " << endl;
//
//  QueryResults ret;
//  for(int i = 0; i < NIMAGES; i++)
//  {
//    db.query(features[i], ret, 4);
//
//    // ret[0] is always the same image in this case, because we added it to the
//    // database. ret[1] is the second best match.
//
//    cout << "Searching for Image " << i << ". " << ret << endl;
//  }
//
//  cout << endl;
//
//  // we can save the database. The created file includes the vocabulary
//  // and the entries added
//  cout << "Saving database..." << endl;
//  db.save("small_db.yml.gz");
//  cout << "... done!" << endl;
//
//  // once saved, we can load it again
//  cout << "Retrieving database once again..." << endl;
//  Surf64Database db2("small_db.yml.gz");
//  cout << "... done! This is: " << endl << db2 << endl;
//}

// ----------------------------------------------------------------------------


