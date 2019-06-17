function normalizeColors(vertices, color) {
    var maxColor = Number.NEGATIVE_INFINITY;
    var minColor = Number.POSITIVE_INFINITY;
    var intensities = [];
    var normalizedIntensities = [];
    var colors = app.cur_pointcloud.geometry.colors;
    k = 0;
    var stride = 4;
    // finds max and min z coordinates
    for ( var i = 0, l = vertices.length / DATA_STRIDE; i < l; i ++ ) {
        if (vertices[ DATA_STRIDE * k + 2] > maxColor) {
            maxColor = vertices[ DATA_STRIDE * k + 2];
        }
        if (vertices[ DATA_STRIDE * k + 2] < minColor) {
            minColor = vertices[ DATA_STRIDE * k + 2];
        }
        intensities.push(vertices[ DATA_STRIDE * k + 2]);
        k++;
    }

    mean = calculateMean(intensities);
    sd = standardDeviation(intensities);
    filteredIntensities = filter(intensities, mean, 1 * sd);
    min = getMinElement(filteredIntensities);
    max = getMaxElement(filteredIntensities);
    // normalize colors
    // if greater than 2 sd from mean, set to max color
    // if less than 2 sd from mean, set to min color
    // ortherwise normalize color based on min and max z-coordinates
    var intensity;
    for ( var i = 0;  i < app.cur_pointcloud.geometry.vertices.length; i ++ ) {
        intensity = intensities[i];
        if (i < intensities.length) {
            if (intensities[i] - mean >= 2 * sd) {
                intensity = 1;
            } else if (mean - intensities[i] >= 2 * sd) {
                intensity = 0;
            } else {
                intensity = (intensities[i] - min) / (max - min);
            }
        } else {
            intensity = 0;
        }
        colors[i].setRGB(intensity, 0, 1 - intensity);
        colors[i].multiplyScalar(intensity * 5);    
        app.cur_pointcloud.geometry.colorsNeedUpdate = true;
    }
    
    return colors;
}

function highlightPoints(indices) {
    var pointcloud = app.cur_pointcloud;
    for (var j = 0; j < indices.length; j++) {
        pointcloud.geometry.colors[indices[j]] = new THREE.Color( 0,1,0 );
    }
    pointcloud.geometry.colorsNeedUpdate = true;

}

function clearScene() {
    var to_remove = [];

    scene.traverse ( function( child ) {
        if ( child instanceof THREE.Mesh && !child.userData.keepMe === true ) {
            to_remove.push( child );
         }
    } );

    for ( var i = 0; i < to_remove.length; i++ ) {
        scene.remove( to_remove[i] );
    }
}


function generateNewPointCloud( vertices, color ) {
    var geometry = new THREE.Geometry();
    var colors = [];
    var k = 0;
    for ( var i = 0, l = vertices.length / DATA_STRIDE; i < l; i ++ ) {
        // creates new vector from a cluster and adds to geometry
        var v = new THREE.Vector3( vertices[ DATA_STRIDE * k + 1 ], 
            vertices[ DATA_STRIDE * k + 2 ], vertices[ DATA_STRIDE * k ] );

        // stores y coordinates into yCoords
        // app.cur_frame.ys.push(vertices[ DATA_STRIDE * k + 2 ]);
        
        // add vertex to geometry
        geometry.vertices.push( v );
        colors.push(color.clone());
        k++;
    }
    geometry.colors = colors;
    // geometry.colors = colors;
    geometry.computeBoundingBox();

    var material = new THREE.PointsMaterial( { size: pointSize, sizeAttenuation: false, vertexColors: THREE.VertexColors } );
    // creates pointcloud given vectors
    var pointcloud = new THREE.Points( geometry, material );
    app.cur_pointcloud = pointcloud;
    normalizeColors(vertices, color);
    return pointcloud;
}

function make_predicted_bboxes(scene,array ){

    scene.traverse ( function( child ) {
        if ( child.name == "lines" ) {
            scene.remove( child );
         }
    } );

    var group = new THREE.Object3D();
    var material = new THREE.LineBasicMaterial( { color: 0xffffff,linewidth: 1 } );

    gt_boxes3d = array
    num = gt_boxes3d.length
    console.log(num)
    console.log(array)
    var i = 0
    var j = 0
    for (n = 0; n < num; n++) {
        var b = gt_boxes3d[n]

    for (k = 0; k < 4; k++) {
        var geometry1 = new THREE.Geometry();
        var geometry2 = new THREE.Geometry();
        var geometry3 = new THREE.Geometry();


        i = k
        j=(k+1)%4
        geometry1.vertices.push(new THREE.Vector3( b[i][0], b[i][1], b[i][2]) );
        geometry1.vertices.push(new THREE.Vector3( b[j][0], b[j][1], b[j][2]) );
        var line1 = new THREE.Line( geometry1, material );

        group.add( line1 );
        // console.log(i);
        // console.log(j);
        // console.log( b[i][0], b[i][1], b[i][2]);
        // console.log( b[j][0], b[j][1], b[j][2]);
        // console.log(geometry1.vertices);
        // console.log("one");

        i = k+4
        j = (k+1)%4 + 4
        geometry2.vertices.push(new THREE.Vector3( b[i][0], b[i][1], b[i][2]) );
        geometry2.vertices.push(new THREE.Vector3( b[j][0], b[j][1], b[j][2]) );
        var line2 = new THREE.Line( geometry2, material );

                group.add( line2 );

        // console.log(i);
        // console.log(j);
        // console.log( b[i][0], b[i][1], b[i][2]);
        // console.log( b[j][0], b[j][1], b[j][2]);
        // console.log(geometry2.vertices);
        // console.log("two");


        i = k
        j = k+4
        geometry3.vertices.push(new THREE.Vector3( b[i][0], b[i][1], b[i][2]) );
        geometry3.vertices.push(new THREE.Vector3( b[j][0], b[j][1], b[j][2]) );
        var line3 = new THREE.Line( geometry3, material );

        group.add( line3 );



        // console.log(i);
        // console.log(j);
        // console.log( b[i][0], b[i][1], b[i][2]);
        // console.log( b[j][0], b[j][1], b[j][2]);
        // console.log(geometry3.vertices);
        // console.log("three");

    } 

    }
    group.name = "lines"
    scene.add(group);
}

function updatePointCloud( vertices, color ) {
    var k = 0;
    var n = vertices.length;
    var l = app.cur_pointcloud.geometry.vertices.length;
    var geometry = app.cur_pointcloud.geometry
    var v;
    for ( var i = 0; i < n / DATA_STRIDE; i ++ ) {
        if (i >= l) {
            v = new THREE.Vector3( vertices[ DATA_STRIDE * k + 1 ], 
                app.cur_frame.ys[k], vertices[ DATA_STRIDE * k ] );
            geometry.vertices.push(v);
            geometry.colors.push(color.clone());



            // stores y coordinates into yCoords
            // app.cur_frame.ys.push(vertices[ DATA_STRIDE * k + 2 ]);
            geometry.verticesNeedUpdate = true;
            geometry.colorsNeedUpdate = true;
        } else {
            v = geometry.vertices[k];
            v.setX(vertices[ DATA_STRIDE * k + 1 ]);
            v.setY(app.cur_frame.ys[k]);
            v.setZ(vertices[ DATA_STRIDE * k ]);
            geometry.verticesNeedUpdate = true;
        }
        k++;
    }
    normalizeColors(vertices, null);
    geometry.computeBoundingBox();
    console.log("fii");
    if (app.cur_frame != null && app.cur_frame.mask_rcnn_indices.length > 0) {
        highlightPoints(app.cur_frame.mask_rcnn_indices);
        console.log("highlighted")
    }

    return app.cur_pointcloud;

}