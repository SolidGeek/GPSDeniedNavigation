#include "opticalflow.h"

OpticalFlow::OpticalFlow(){
}

OpticalFlow::OpticalFlow( int frame_width, int frame_height, int scaledown, int interval )
{

    // Initialize feature tracker and track 100 points
    this->tracker = new FeatureTracker( this->status, 100 );

    int scale_width = frame_width/scaledown;
    int scale_height = frame_height/scaledown;

    // Calculate scaled size
    this->size = cv::Size(scale_width, scale_height);

    this->interval = interval;

    // Create fixed grid for sparse method
    for ( int x = 0; x < scale_width; x += this->interval ){
        for ( int y = 0; y < scale_height; y += this->interval ){
            this->grid.push_back( cv::Point2f(x,y) );
        }
    }
}

void OpticalFlow::process_frame( cv::Mat raw, cv::Mat &output ){
    // Resize the frame to specified size
    cv::resize(raw, this->frame, this->size);

    // Convert to grayscale (and use this to perform flow analysis)
    cv::cvtColor(this->frame, output, cv::COLOR_BGR2GRAY);
}

cv::Point2f OpticalFlow::compute_dense_flow( cv::Mat raw, float dt, float distance )
{
    cv::Mat vectors;    // Calculated flow vectors
    cv::Mat gray;       // Output image for processing

    float xsum, ysum = 0;
    float xflow, yflow;
    int count = 0;

    // Convert raw frame to scaled down grayscale
    this->process_frame( raw, gray );
    
    if( !this->prev_gray.empty() ){

        // Use opencv to calculate flow between new frame and previous frame
        cv::calcOpticalFlowFarneback( this->prev_gray, gray, vectors, this->pyr_scale, this->levels, this->winsize, this->iterations, this->poly_n, this->poly_sigma, 0);

        for (int y = 0; y < vectors.rows; y+=this->interval)
        {
            for (int x = 0; x < vectors.cols; x+=this->interval)
            {
                // Extract the vector the flow in this 
                cv::Point2f vector = vectors.at<cv::Point2f>(y,x);

                xsum += vector.x;
                ysum += vector.y;

                // Visualize the flow in the frame
                cv::line(this->frame, cv::Point(x,y), cv::Point(x+int(vector.x),y+int(vector.y)), cv::Scalar(0, 255, 0) );
                cv::circle(this->frame, cv::Point(x,y), 1, cv::Scalar(0, 255, 0) );

                count++;
            }
        }

        // Calculate flow from sums
        this->flow.x = this->compute_pixel_velocity( xsum, count, dt);
        this->flow.y = this->compute_pixel_velocity( ysum, count, dt);
    }

    this->prev_gray = gray;

    if( distance != 0 ){

        this->velocity.x = this->compute_velocity( this->flow.x, gray.cols, distance );
        this->velocity.x = this->compute_velocity( this->flow.x, gray.rows, distance );

        return this->velocity;
    }

    return this->flow;
}

cv::Point2f OpticalFlow::compute_sparse_flow( cv::Mat raw, float dt, float distance ){

    cv::Mat vectors;    // Calculated flow vectors
    cv::Mat gray;       // Output image for processing
    std::vector<uchar> status;
    std::vector<float> errors;

    float xsum, ysum = 0;
    float xflow, yflow;
    int count = 0;

    // Convert raw frame to scaled down grayscale
    this->process_frame( raw, gray );

    if( !this->prev_gray.empty() ){

        cv::calcOpticalFlowPyrLK( this->prev_gray, gray, this->grid, vectors, status, errors, cv::Size(21, 21), 3 );
        this->grid = vectors;

        for (int y = 0; y < vectors.rows; y++)
        {
            for (int x = 0; x < vectors.cols; x++)
            {
                cv::Point2f start   = grid.at<cv::Point2f>(y,x);
                cv::Point2f end     = vectors.at<cv::Point2f>(y,x);
       
                if( status[count] == 1 ){
                    
                    if( errors[count] < 2.5f )
                        // printf( "%.2f", errors[count] );
                        // Visualize the flow in the frame
                        cv::line(this->frame, start, end, cv::Scalar(0, 255, 0) );
                }
                
                cv::circle(this->frame, start, 1, cv::Scalar(0, 255, 0) );

                count++;
            }
        }

    }

    this->prev_gray = gray;

    if( distance != 0 ){

        /*  this->velocity.x = this->compute_velocity( this->flow.x, gray.cols, distance );
        this->velocity.x = this->compute_velocity( this->flow.x, gray.rows, distance );

        return this->velocity; */
    }

    return this->flow;

}


cv::Point2f OpticalFlow::compute_flow_features( cv::Mat raw, float dt, float distance ){

    cv::Mat gray; // Output image for processing
    
    /* if (status.empty()) {
		status.resize(100, 2); // Request = 2 features = 30 
	}*/

    float xsum, ysum = 0;
    int count = 0;

    

    // Convert raw frame to scaled down grayscale
    this->process_frame( raw, gray );

    // trackFeatures are made for stereo, so first two arguments is left and right image. 
    tracker->track_features(gray, this->features_current, this->status );

    if (!this->features_current.empty() && !this->features_previous.empty()) {
		//calculate pixel flow

		for (int i = 0; i < this->status.size(); i++) {
			//just use active features
			if ( status[i] == 1 ) {
				xsum += features_current[i].x - features_previous[i].x;
				ysum += features_current[i].y - features_previous[i].y;

                // Visualize the flow in the frame
                cv::line(this->frame, cv::Point(features_previous[i].x,features_previous[i].y), cv::Point(features_current[i].x, features_current[i].y), cv::Scalar(0, 255, 0) );
                cv::circle(this->frame, cv::Point(features_current[i].x,features_current[i].y), 1, cv::Scalar(0, 255, 0) );

				count++;
			}
		}

        // Calculate flow from sums
        this->flow.x = this->compute_pixel_velocity( xsum, count, dt);
        this->flow.y = this->compute_pixel_velocity( ysum, count, dt);

	}

    this->features_previous = this->features_current;

    tracker->update_feature_status( this->status );

    if( distance != 0 ){

        this->velocity.x = this->compute_velocity( this->flow.x, gray.cols, distance );
        this->velocity.y = this->compute_velocity( this->flow.y, gray.rows, distance );

        return this->velocity;
    }

    return this->flow;

}

float OpticalFlow::compute_pixel_velocity( float sum, int count, float dt ){
    // Calculate pixels per second
    return (sum / (float)count) / dt;
}

float OpticalFlow::compute_velocity( float flow, int axis_length, float distance ){

    int center_distance = axis_length/2;

    float pixel_per_length = (center_distance / (float)distance);

    return (flow / pixel_per_length);

}

cv::Mat OpticalFlow::get_frame( void ){
    return this->frame;
}