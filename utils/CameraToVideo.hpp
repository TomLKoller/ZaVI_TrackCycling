/*
 * CameraToVideo.hpp
 *
 *  Created on: 07.02.2019
 *      Author: tomlucas
 */

#ifndef CAMERATOVIDEO_HPP_
#define CAMERATOVIDEO_HPP_

#include <osg/Camera>
#include <osgDB/OutputStream>
#include <osgDB/ImageOptions>
#include <osgDB/WriteFile>
#include <osgViewer/Viewer>
#include "threaded_object.hpp"
#include "ZaVI_Utils.hpp"
#include "Callbacker.hpp"
#include <iomanip>
namespace zavi
::osg_util {
	/**
	 * Utility class to save the image in the osg viewer
	 *
	 * add this class's camera_callback to the osgViewer via addPostDrawCallback to read the images
	 *
	 * add this class as a callback to save the images on notifications or start it as a threaded object to save periodícally to a file
	 *
	 * adding it to the osv Viewer wont save anything on the hard drive
	 */
	class CameraToVideo: public ThreadedObject, public Callback {
	private:
		/**
		 * Inner struct to decouple osg::ref_ptr and std::shared_ptr
		 */
		struct InnerCallback: public osg::Camera::DrawCallback {
			InnerCallback():image(new ::osg::Image()) {

			}
			/**
			 * osg callback
			 * @param renderInfo
			 */
			virtual void operator() (osg::RenderInfo &renderInfo) const {
				osg::Camera * camera = renderInfo.getCurrentCamera();
				int width = camera -> getViewport() -> width();
				int height = camera -> getViewport() -> height();

				image->readPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE);

			}

			osg::ref_ptr<::osg::Image> image;		//!< the current image of the osg Viewer
		};

		std::string filename;     //!< filename to save images to (prefix)
		osgDB::OutputStream outstream;//!< the outstream images will be saved to ./images/prefixXXXXX
		int frame=0;//!< frame number
		int stride;//!< each <stride>th frame will be saved
		int counter;//!< count callback calls
	public:

		osg::ref_ptr<InnerCallback> camera_callback;//!< the callback for the osg viewer

		virtual void run(double freq, bool (*running)()) {
			Timer loop(freq);
			while (running()) {

				loop.wait();
				writeImage();

			}
		}
		/**
		 * write the current image to the outsstream
		 */
		void writeImage() {
			std::ostringstream ostr;
			ostr << "images/";
			ostr << filename;
			ostr << std::setfill('0')<< std::setw(6) <<frame;
			ostr << ".png";
			std::string result=ostr.str();
			osgDB::writeImageFile(*(camera_callback->image.get()),result);
			frame++;

		}

		/**
		 * Constructor, builds the outstream
		 * @param filename the prefix for the output files
		 */
		CameraToVideo(osgViewer::Viewer &viewer, const char * filename, int stride=1):filename(filename),outstream(new osgDB::ImageOptions()),stride(stride), counter(0), camera_callback(new InnerCallback()) {
			outstream.setWriteImageHint(osgDB::OutputStream::WRITE_EXTERNAL_FILE);
			LOG(INFO)<<"Writing images to "<<filename << "xxxxxx";
			viewer.getCamera()->addPostDrawCallback(camera_callback);
		}
		/**
		 * zavi callback
		 *
		 * calls writeImage
		 */
		virtual void operator() (void * callbacker) {
			counter++;
			if(counter>=stride) {
				writeImage();
				counter=0;
			}
		}

	}
	;
}

#endif /* CAMERATOVIDEO_HPP_ */
