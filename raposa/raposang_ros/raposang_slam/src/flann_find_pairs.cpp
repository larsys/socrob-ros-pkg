#include <raposang_slam.h>

void matchingORB(const CvSeq* sd, const CvSeq* id, std::vector<int> &pairs){

	int i, j, k, d1, d2, jj, dist, i_total, s_total;
	unsigned char *i_ptr, *s_ptr;
	
	CvSeqReader i_reader;
	CvSeqReader s_reader;
	
	i_total = id->total;
	s_total = sd->total;
		
	cvStartReadSeq(sd, &s_reader, 0);		
	s_ptr = (unsigned char*) s_reader.ptr;
	
	for(i=0;i<s_total;i++){
	
		d1 = d2 = 0;
		jj = -1;
	
		cvStartReadSeq(id, &i_reader, 0);		
		i_ptr = (unsigned char*) i_reader.ptr;
		
		for(j=0;j<i_total;j++){
			
			dist = 0;
			
			for(k=0;k<32;k++)
				dist += (s_ptr[k]==i_ptr[k]) ? 1 : 0;
	
			if(dist>d1){
				d2 = d1;
				d1 = dist;
				jj = j;
			} 
			else if(dist>d2)
				d2 = dist;

			CV_NEXT_SEQ_ELEM(i_reader.seq->elem_size, i_reader);					
			i_ptr = (unsigned char*) i_reader.ptr;	
			
		}		
		
		if((jj>=0) && (d2 < 0.7*d1)){         
			pairs.push_back(i);
			pairs.push_back(jj);			
		}
		
		CV_NEXT_SEQ_ELEM(s_reader.seq->elem_size, s_reader);					
		s_ptr = (unsigned char*) s_reader.ptr;			
		
	}
}

/*void matchpoints(const cv::Mat& desc_s,const cv::Mat& desc_d, std::vector<int> &pairs){
                
	int dist, d1, d2;
	unsigned int i, j;

	for(i = 0; i < desc_s.rows ; i++){
		
		d1=d2=0;
		
		for(j = 0; j < desc_d.rows; j++){
			dist=0;
			
			//hamming distance
			for(int k=0;k<32;k++){
				if(desc_s.at<uchar>(i,k) == desc_d.at<uchar>(j,k))
								dist++;
			}
			
			if(dist>d1){ // if this feature matches better than current best
				d2 = d1;
				d1 = dist;
			}else{
				if(dist>d2)
					d2 = dist;
			}
		
		}
		
		if(d1 > 2*d2){         
			// Store the change in position

			pairs.push_back(i);
			pairs.push_back(j);	
			break;		
		}
	}
}*/


// Code from da webz!

double
compareSURFDescriptors( const float* d1, const float* d2, double best, int length )
{
    double total_cost = 0;
    assert( length % 4 == 0 );
    for( int i = 0; i < length; i += 4 )
    {
        double t0 = d1[i] - d2[i];
        double t1 = d1[i+1] - d2[i+1];
        double t2 = d1[i+2] - d2[i+2];
        double t3 = d1[i+3] - d2[i+3];
        total_cost += t0*t0 + t1*t1 + t2*t2 + t3*t3;
        if( total_cost > best )
            break;
    }
    return total_cost;
}


int
naiveNearestNeighbor( const float* vec, int laplacian,
                      const CvSeq* model_keypoints,
                      const CvSeq* model_descriptors )
{
    int length = (int)(model_descriptors->elem_size/sizeof(float));
    int i, neighbor = -1;
    double d, dist1 = 1e6, dist2 = 1e6;
    CvSeqReader reader, kreader;
    cvStartReadSeq( model_keypoints, &kreader, 0 );
    cvStartReadSeq( model_descriptors, &reader, 0 );

    for( i = 0; i < model_descriptors->total; i++ )
    {
        const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
        const float* mvec = (const float*)reader.ptr;
    	CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
        CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
        if( laplacian != kp->laplacian )
            continue;
        d = compareSURFDescriptors( vec, mvec, dist2, length );
        if( d < dist1 )
        {
            dist2 = dist1;
            dist1 = d;
            neighbor = i;
        }
        else if ( d < dist2 )
            dist2 = d;
    }
    if ( dist1 < 0.6*dist2 )
        return neighbor;
    return -1;
}

void
findPairs( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
           const CvSeq* imageKeypoints, const CvSeq* imageDescriptors, vector<int>& ptpairs )
{
    int i;
    CvSeqReader reader, kreader;
    cvStartReadSeq( objectKeypoints, &kreader );
    cvStartReadSeq( objectDescriptors, &reader );
    ptpairs.clear();

    for( i = 0; i < objectDescriptors->total; i++ )
    {
        const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
        const float* descriptor = (const float*)reader.ptr;
        CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
        CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
        int nearest_neighbor = naiveNearestNeighbor( descriptor, kp->laplacian, imageKeypoints, imageDescriptors );
        if( nearest_neighbor >= 0 )
        {
            ptpairs.push_back(i);
            ptpairs.push_back(nearest_neighbor);
        }
    }
}

void flannFindPairs( const CvSeq*, const CvSeq* objectDescriptors,
           const CvSeq*, const CvSeq* imageDescriptors, vector<int>& ptpairs ) {
           
	int length = (int)(objectDescriptors->elem_size/sizeof(float));

    cv::Mat m_object(objectDescriptors->total, length, CV_32F);
	cv::Mat m_image(imageDescriptors->total, length, CV_32F);


	// copy descriptors
    CvSeqReader obj_reader;
	float* obj_ptr = m_object.ptr<float>(0);
    cvStartReadSeq( objectDescriptors, &obj_reader );
    for(int i = 0; i < objectDescriptors->total; i++ )
    {
        const float* descriptor = (const float*)obj_reader.ptr;
        CV_NEXT_SEQ_ELEM( obj_reader.seq->elem_size, obj_reader );
        memcpy(obj_ptr, descriptor, length*sizeof(float));
        obj_ptr += length;
    }
    CvSeqReader img_reader;
	float* img_ptr = m_image.ptr<float>(0);
    cvStartReadSeq( imageDescriptors, &img_reader );
    for(int i = 0; i < imageDescriptors->total; i++ )
    {
        const float* descriptor = (const float*)img_reader.ptr;
        CV_NEXT_SEQ_ELEM( img_reader.seq->elem_size, img_reader );
        memcpy(img_ptr, descriptor, length*sizeof(float));
        img_ptr += length;
    }

    // find nearest neighbors using FLANN
    cv::Mat m_indices(objectDescriptors->total, 2, CV_32S);
    cv::Mat m_dists(objectDescriptors->total, 2, CV_32F);
    cv::flann::Index flann_index(m_image, cv::flann::KDTreeIndexParams(4));  // using 4 randomized kdtrees
    flann_index.knnSearch(m_object, m_indices, m_dists, 2, cv::flann::SearchParams(64) ); // maximum number of leafs checked

    int* indices_ptr = m_indices.ptr<int>(0);
    float* dists_ptr = m_dists.ptr<float>(0);
    for (int i=0;i<m_indices.rows;++i) {
    	if (dists_ptr[2*i]<0.6*dists_ptr[2*i+1]) {
    		ptpairs.push_back(i);
    		ptpairs.push_back(indices_ptr[2*i]);
    	}
    }
}
