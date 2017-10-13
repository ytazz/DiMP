#include <window/camera.h>
#include <module/rendering.h>
#include <module/module.h>

///////////////////////////////////////////////////////////////////////////////////////////////////

void Camera::Read(XMLNode* node){
	node->Get(fov      , ".fov"      );
	node->Get(latitude , ".latitude" );
	node->Get(longitude, ".longitude");
	node->Get(distance , ".distance" );
	node->Get(distMin  , ".dist_min" );
	node->Get(distMax  , ".dist_max" );

	vec3_t t;
	node->Get(t, ".target"   );
	target = t;
}
