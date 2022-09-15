import * as THREE from 'three';
import metaversefile from 'metaversefile';
import { Vector3 } from 'three';

const {useApp, useFrame, useLoaders, usePhysics, useCleanup, useLocalPlayer, useActivate, useScene, useInternals} = metaversefile;

const baseUrl = import.meta.url.replace(/(\/)[^\/\/]*$/, '$1'); 

const localVector = new THREE.Vector3();
const localVector2 = new THREE.Vector3();
const localVector3 = new THREE.Vector3();
const localVector4 = new THREE.Vector3();
const localVector5 = new THREE.Vector3();
const localQuaternion = new THREE.Quaternion();
const localQuaternion2 = new THREE.Quaternion();
const localQuaternion3 = new THREE.Quaternion();
const localEuler = new THREE.Euler();
const localMatrix = new THREE.Matrix4();
window.isDebug = false


export default () => {  

    const app = useApp();
    window.heli = app
    const physics = usePhysics();
    window.physics = physics;
    const scene = useScene();
    const physicsIds = [];
    const localPlayer = useLocalPlayer();
    const {camera} = useInternals();

    let vehicleObj;

    //let velocity = new THREE.Vector3();
    //let angularVelocity = new THREE.Vector3();
    let vehicle = null;
    let yaw = 0;
    let roll = 0;
    let pitch = 0;
    let enginePower = 0;
    let powerFactor = 0.03;
    let damping =1;
    let rotor = null;
    let sitSpec = null;

    // Inputs
    let keyW = false;
    let keyA = false;
    let keyS = false;
    let keyD = false;
    let keyShift = false;
    let keyQ = false;
    let keyE = false;
    let keyC = false;
    let keyX = false;

    let sitPos = null;

    let sitAnim = null;
    let rayArray = [];
    let wheelArray = [];
    let pointArray = [];
    let sceneWheels = [];
    let moveSpeed = 0;
    let newRot = 0;

    // SFX
    let engineSound = null;
    let crashSound = null;

    //Suspension

    let suspensionMaxLength = 0.6;
    let wheelRadius = 0.3;
    let stiffness = 300;
    let damper = 2.5;
    let suspensionLengthArray = [];
    let wheelFriction = 1;
    let brakeForce = 250;
    let steeringAngle = 35;
    let actualSpeed = 0;
    let lastDrag = 0;
    let tempLinear = new THREE.Vector3();

    // Prop

    let propeller = null;
    let aileronL = null;
    let aileronR = null;

    let rudder = null;
    let elevatorL = null;
    let elevatorR = null;

    let aileronMaxAngle = 35;
    let rudderMaxAngle = 5;
    let elevatorMaxAngle = 35;

    let linearVelocity = new THREE.Vector3();
    let angularVelocity = new THREE.Vector3();

    let debugArray = [];
    let debugPointArray = [];


    function onDocumentKeyDown(event) {
        var keyCode = event.which;
        if (keyCode == 87) { // W 
            keyW = true;
        }
        if (keyCode == 83) { // S 
            keyS = true;
        }
        if (keyCode == 65) { // A 
            keyA = true;
        }
        if (keyCode == 68) { // D 
            keyD = true;
        }
        if (keyCode == 69) { // E 
            keyE = true;
        }
        if (keyCode == 81) { // Q 
            keyQ = true;
        }
        if (keyCode == 16) { // L shift 
            keyShift = true;
        }
        if (keyCode == 67) { // C
            keyC = true;
        }
        if (keyCode == 88) { // X
            keyX = true;
        }
    };
    function onDocumentKeyUp(event) {
        var keyCode = event.which;
        if (keyCode == 87) { // W 
            keyW = false;
        }
        if (keyCode == 83) { // S 
            keyS = false;
        }
        if (keyCode == 65) { // A 
            keyA = false;
        }
        if (keyCode == 68) { // D 
            keyD = false;
        }
        if (keyCode == 69) { // E 
            keyE = false;
        }
        if (keyCode == 81) { // Q 
            keyQ = false;
        }
        if (keyCode == 16) { // L shift 
            keyShift = false;
        }
        if (keyCode == 67) { // C
            keyC = false;
        }
        if (keyCode == 88) { // X
            keyX = false;
        }
    };

    const _unwear = () => {
      if (sitSpec) {
        const sitAction = localPlayer.getAction('sit');
        if (sitAction) {
          localPlayer.removeAction('sit');
          // localPlayer.avatar.app.visible = true;
          // physics.setCharacterControllerPosition(localPlayer.characterController, app.position);
          sitSpec = null;
        }
      }
    };

    const loadModel = ( params ) => {

        return new Promise( ( resolve, reject ) => {
                
            const { gltfLoader } = useLoaders();
            gltfLoader.load( params.filePath + params.fileName, function( gltf ) {
                resolve( gltf.scene );     
            });
        })
    }

    const modelName = 'jet.glb';
    let p1 = loadModel( { filePath: baseUrl, fileName: modelName, pos: { x: 0, y: 0, z: 0 } } ).then( result => { vehicleObj = result } );

    let loadPromisesArr = [ p1 ];

    Promise.all( loadPromisesArr ).then( models => {

        app.add( vehicleObj );

        app.setComponent('sit', 
        {
          "key": "sit",
          "value": {
            "subtype": "stand",
            "mountType": "flying",
            "sitOffset": [0, 0, 0],
            "walkAnimation": "",
            "walkAnimationHoldTime": 1,
            "walkAnimationSpeedFactor": 0.1,
            "speed": 0.02,
            "damping": 0.99
          }
        }
        );

        const physicsId = physics.addBoxGeometry(
          new THREE.Vector3(0, 1.5, 0),
          new THREE.Quaternion(),
          new THREE.Vector3(1, 0.1, 1.5), //0.5, 0.05, 1
          true,
          new THREE.Vector3(1,1,0),
          500
        );
        physicsIds.push(physicsId);
        
        vehicle = app.physicsObjects[0];
        window.vehicle = vehicle;
        vehicle.detached = true;

        vehicle.position.copy(app.position)
        physics.setTransform(vehicle);


        app.traverse(o => {
                  
                  if(o.name === "sitPos") {
                    sitPos = o;
                    //console.log(o, "we have a sitPos");
                  }
                   if(o.name === "Prop") {
                    propeller = o;
                    console.log(propeller);
                    //console.log(o, "we have a sitPos");
                  }
                   if(o.name === "rudder") {
                    rudder = o;
                    }
                    if(o.name === "aileronL") {
                    aileronL = o;
                    }
                    if(o.name === "aileronR") {
                    aileronR = o;
                    }
                    if(o.name === "elevatorL") {
                    elevatorL = o;
                    }
                    if(o.name === "elevatorR") {
                    elevatorR = o;
                    }
                  if(o.name === "Glass") {
                    o.material = new THREE.MeshLambertMaterial( { color: 0x000000, opacity: 0.5, transparent: true} );
                    //console.log(o, "we have a sitPos");
                  }
                  if(o.name === "originF") {
                    rayArray[0] = o;

                  }
                  if(o.name === "originBL") {
                    rayArray[1] = o;
                    
                  }
                  if(o.name === "originBR") {
                    rayArray[2] = o;
                  }
                  if(o.name === "front") {
                    wheelArray[0] = o;

                  }
                  if(o.name === "backL") {
                    wheelArray[1] = o;
                    
                  }
                  if(o.name === "backR") {
                    wheelArray[2] = o;
                  }
                  //console.log(rayArray);
                  o.castShadow = true;
                });

                for (var i = 0; i < wheelArray.length; i++) {
                  let dum = new THREE.Object3D;
                  dum = wheelArray[i].clone();
                  scene.add(dum);
                  sceneWheels[i] = dum;
                  sceneWheels[i].updateMatrixWorld();
                  wheelArray[i].visible = false;

                  const geometry = new THREE.BoxGeometry( .1, .1, .1 );
                  let material = new THREE.MeshBasicMaterial( {color: 0x00ff00} );

                  const geometry2 = new THREE.BoxGeometry( .35, .35, .35 );
                  let material2 = new THREE.MeshBasicMaterial( {color: 0x00ff00} );
                  

                  if(i === 0) {
                    material = new THREE.MeshBasicMaterial( {color: 0xff0000} );
                    material2 = new THREE.MeshBasicMaterial( {color: 0xff0000} );
                  }

                  const cube = new THREE.Mesh( geometry, material );
                  const cube2 = new THREE.Mesh( geometry2, material2 );

                  
                  //scene.add( cube );
                  //scene.add( cube2 );
                  //debugArray.push(cube);
                  //debugPointArray.push(cube2);
                }

               const listener = new THREE.AudioListener();
               camera.add( listener );

               engineSound = new THREE.PositionalAudio( listener );
               const audioLoader = new THREE.AudioLoader();
                // audioLoader.load( baseUrl + 'plane/idle.wav', function( buffer ) {
                //       engineSound.setBuffer( buffer );
                //       engineSound.setRefDistance( 5 );
                //       engineSound.setVolume( 0.5 );
                //       engineSound.setLoop(true);
                      
                // });

                app.add(engineSound);

                suspensionLengthArray[0] = 0.5;
                suspensionLengthArray[1] = 0.5;
                suspensionLengthArray[2] = 0.5;
                suspensionLengthArray[3] = 0.5;

                app.updateMatrixWorld();
    });
    useFrame(( { timeDiff } ) => {

      const _updateEngine = () => {
        if(engineSound) {

            const rpmStartPoint = -500;
            var spd = rpmStartPoint + (moveSpeed*30);
            engineSound.setDetune(spd);

        }

        if(propeller) {
            propeller.rotateZ(moveSpeed*0.1);
        }
      }

      const _updateSuspension = () => {

      }

      const _updateControlSurfaces = () => {
        if(rudder) {
            if(keyQ) {
                rudder.rotation.y = THREE.Math.degToRad(-rudderMaxAngle)
            }
            else if(keyE) {
                rudder.rotation.y = THREE.Math.degToRad(rudderMaxAngle)
            }
            else {
                rudder.rotation.y = 0;
            }
        }
        if(elevatorL && elevatorR) {
            if(keyW) {
                elevatorL.rotation.x = THREE.Math.degToRad(elevatorMaxAngle)
                elevatorR.rotation.x = THREE.Math.degToRad(215)
            }
            else if(keyS) {
                elevatorL.rotation.x = THREE.Math.degToRad(-elevatorMaxAngle)
                elevatorR.rotation.x = THREE.Math.degToRad(145)
            }
            else {
                elevatorL.rotation.x = 0;
                elevatorR.rotation.x = THREE.Math.degToRad(180);
            }
        }
        /*if(aileronL && aileronR) {
            console.log("we got aileronL");
            if(keyA) {
                aileronL.rotation.x = THREE.Math.degToRad(23.0711);
                aileronL.rotation.y = THREE.Math.degToRad(-0.881149);
                aileronL.rotation.z = THREE.Math.degToRad(-6.11615);

                aileronR.rotation.x = THREE.Math.degToRad(-47.2);
                aileronR.rotation.y = THREE.Math.degToRad(0.321865);
                aileronR.rotation.z = THREE.Math.degToRad(6.41496);
                
            }
            else if(keyD) {
                aileronL.rotation.x = THREE.Math.degToRad(-47.2);
                aileronL.rotation.y = THREE.Math.degToRad(0.321865);
                aileronL.rotation.z = THREE.Math.degToRad(-6.41496);

                aileronR.rotation.x = THREE.Math.degToRad(-47.2);
                aileronR.rotation.y = THREE.Math.degToRad(0.321865);
                aileronR.rotation.z = THREE.Math.degToRad(6.41496);

               
            }
            else {
                aileronL.rotation.x = THREE.Math.degToRad(-47.2);
                aileronL.rotation.y = THREE.Math.degToRad(0.321865);
                aileronL.rotation.z = THREE.Math.degToRad(-6.41496);

                aileronR.rotation.x = THREE.Math.degToRad(-47.2);
                aileronR.rotation.y = THREE.Math.degToRad(0.321865);
                aileronR.rotation.z = THREE.Math.degToRad(6.41496);

               
            }
        }*/
      }

      const _brake = (index) => {

        let brakeForce = new THREE.Vector3(0,0,-5).applyQuaternion(vehicle.quaternion);
        let targetPos = new THREE.Vector3();
        rayArray[index].getWorldPosition(targetPos);
        physics.addForceAtPos(vehicle, brakeForce.multiplyScalar(0.005), targetPos);

      }

      const _updateRide = () => {

        if (rayArray.length > 0 && rayArray) {
          const {instanceId} = app;

          //physics.disableGeometryQueries(vehicle);

          if(rayArray) {
            if(rayArray) {
              let quat = new THREE.Quaternion(vehicle.quaternion.x, vehicle.quaternion.y, vehicle.quaternion.z, vehicle.quaternion.w);
              let right = new THREE.Vector3(1, 0, 0).applyQuaternion(quat);
              let globalUp = new THREE.Vector3(0, 1, 0);
              let up = new THREE.Vector3(0, 1, 0).applyQuaternion(quat);
              let forward = new THREE.Vector3(0, 0, 1).applyQuaternion(quat);
              let angularInput = new THREE.Vector3();

              if(engineSound && !engineSound.isPlaying) {
                engineSound.play();
              }
              
                enginePower = 1;

                let lookVelocity = new THREE.Vector3();
                physics.getLinearVelocity(vehicle, linearVelocity);
                physics.getAngularVelocity(vehicle, angularInput);
                physics.getLinearVelocity(vehicle, lookVelocity);
                lookVelocity.normalize();
                let rotStabVelocity = new THREE.Quaternion().setFromUnitVectors(forward, lookVelocity);
                rotStabVelocity.x *= 0.3;
                rotStabVelocity.y *= 0.3;
                rotStabVelocity.z *= 0.3;
                rotStabVelocity.w *= 0.3;
                let rotStabEuler = new THREE.Euler().setFromQuaternion(rotStabVelocity);

                let rotStabInfluence = THREE.MathUtils.clamp(lookVelocity.length() - 1, 0, 0.1);  // Only with speed greater than 1 UPS
                rotStabInfluence *= 0;
                let loopFix = (keyShift && actualSpeed > 0 ? 0 : 1);
                
                angularInput.x += rotStabEuler.x * rotStabInfluence * loopFix;
                angularInput.y += rotStabEuler.y * rotStabInfluence;
                angularInput.z += rotStabEuler.z * rotStabInfluence * loopFix;

              // IO
              if(keyShift) {
                if(moveSpeed < 100) {
                  moveSpeed += 1;
                }
              }
              if(keyX) {
                if(moveSpeed > 0) {
                  moveSpeed -= 1;
                }
              }
              //console.log(moveSpeed, actualSpeed);
              if(keyW) {
                angularInput.x += right.x * powerFactor * enginePower;
                angularInput.y += right.y * powerFactor * enginePower;
                angularInput.z += right.z * powerFactor * enginePower;
              }
              if(!keyW && actualSpeed < 0) {
                //moveSpeed += powerFactor*2 * enginePower*5;
              }
              if (keyS) {
                //angularInput.x = up.x - .5;
                angularInput.x -= right.x * powerFactor * enginePower;
                angularInput.y -= right.y * powerFactor * enginePower;
                angularInput.z -= right.z * powerFactor * enginePower;
              }
              if(keyQ) {
                //newRot += powerFactor * moveSpeed/10;
                angularInput.x += up.x * powerFactor/2 * enginePower;
                angularInput.y += up.y * powerFactor/2 * enginePower
                angularInput.z += up.z * powerFactor/2 * enginePower;
                //newRot = steeringAngle;
              }
              if (keyE) {
                //newRot -= powerFactor * moveSpeed/10;
                angularInput.x -= up.x * powerFactor/2 * enginePower;
                angularInput.y -= up.y * powerFactor/2 * enginePower
                angularInput.z -= up.z * powerFactor/2 * enginePower;
                //newRot = -steeringAngle;
              }
              if(keyA) {
                //newRot += powerFactor * moveSpeed/10;
                angularInput.x -= forward.x * powerFactor * enginePower;
                angularInput.y -= forward.y * powerFactor * enginePower
                angularInput.z -= forward.z * powerFactor * enginePower;
                //newRot = steeringAngle;
              }
              if (keyD) {
                //newRot -= powerFactor * moveSpeed/10;
                angularInput.x += forward.x * powerFactor * enginePower;
                angularInput.y += forward.y * powerFactor * enginePower
                angularInput.z += forward.z * powerFactor * enginePower;
                //newRot = -steeringAngle;
              }
              if (keyShift) {
                if(moveSpeed > 0) {
                    //moveSpeed -= powerFactor*2 * brakeForce;
                }
              }


              //linearVelocity.copy(lookVelocity);
             linearVelocity.x += (linearVelocity.length() * lastDrag + moveSpeed/1000) * forward.x * enginePower;
             linearVelocity.y += (linearVelocity.length() * lastDrag + moveSpeed/1000) * forward.y * enginePower;
             linearVelocity.z += (linearVelocity.length() * lastDrag + moveSpeed/1000) * forward.z * enginePower;

              //let velLength2 = body.velocity.length();
             const drag = Math.pow(lookVelocity.length(), 1) * 0.03 * enginePower;
             
             linearVelocity.x -= linearVelocity.x * drag;
             linearVelocity.y -= linearVelocity.y * drag;
             linearVelocity.z -= linearVelocity.z * drag;
             lastDrag = drag;

             let lift = Math.pow(linearVelocity.length(), 1) * 0.005 * enginePower;
             lift = THREE.MathUtils.clamp(lift, 0, 0.05);
             linearVelocity.x += up.x * lift;
             linearVelocity.y += up.y * lift;
             linearVelocity.z += up.z * lift;

             angularInput.x = THREE.MathUtils.lerp(angularInput.x, angularInput.x * 0.98, 1);
             angularInput.y = THREE.MathUtils.lerp(angularInput.y, angularInput.y * 0.98, 1);
             angularInput.z = THREE.MathUtils.lerp(angularInput.z, angularInput.z * 0.98, 1);

              //moveSpeed *= 0.99;
              newRot *= 0.9;

              const downQuat = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI*0.5);
              let target = new THREE.Vector3();
              let target2 = new THREE.Vector3();
              let target3 = new THREE.Vector3();
               rayArray[0].getWorldPosition( target );
               rayArray[1].getWorldPosition( target2 );
               rayArray[2].getWorldPosition( target3 );
              

               pointArray[0] = physics.raycast(target, downQuat);
               pointArray[1] = physics.raycast(target2, downQuat);
               pointArray[2] = physics.raycast(target3, downQuat);
               

               //console.log(wheelArray.length, rayArray.length, pointArray.length);

              for (var i = 0; i < rayArray.length; i++) {

                    var dir = new THREE.Vector3(); // create once an reuse it
                    let v1 = rayArray[i].position.clone();
                    let targetss = new THREE.Vector3();
                    rayArray[i].getWorldPosition(targetss);
                    let targetss2 = new THREE.Vector3();
                    wheelArray[i].getWorldPosition(targetss2);
                    let v2 = new THREE.Vector3().fromArray(pointArray[i].point);
                    dir.subVectors(targetss, v2);
                    //velocity.add(dir);
                    let force = 0;
                    let yOffset = 0;
                    force = Math.abs(1 / ((pointArray[i].point[1] + 0.4) - targetss.y))
                    //force * (1/pointArray[i].distance + 3);
                    //console.log(force);
                    let newVec = new THREE.Vector3(0,force*2,0);

                    let tempLinear = new THREE.Vector3();
                    let tempAngular = new THREE.Vector3();
                    physics.getLinearVelocity(vehicle, tempLinear);
                    physics.getAngularVelocity(vehicle, tempAngular);

                    let crossedVec = tempAngular.cross(new THREE.Vector3().fromArray(pointArray[i].point).sub(vehicle.position));

                    let pointVel = tempLinear.add(crossedVec);

                    // debug

                    //debugArray[i].position.copy(targetss);
                    //debugArray[i].updateMatrixWorld();
                    
                    //debugPointArray[i].position.copy(new THREE.Vector3().fromArray(pointArray[i].point));
                    //debugPointArray[i].updateMatrixWorld();

                    let fx = 0;
                    let fy = 0;

                    let newRotVec = new THREE.Vector3(0,0,0);
                    let newPointVec = new THREE.Vector3(0,0,0);
                    var clampedRot = THREE.Math.clamp(newRot, -steeringAngle, steeringAngle)
                    fx = clampedRot / 2;
                    fy = pointVel.x;

                    newRotVec.x = fx * right.x;
                    newRotVec.y = fx * right.y;
                    newRotVec.z = fx * right.z;

                    newPointVec.x = fy * -up.x;
                    newPointVec.y = fy * -up.y;
                    newPointVec.z = fy * -up.z;

                    let rayDistance = new THREE.Vector3();
                    let pointVector =  new THREE.Vector3().fromArray(pointArray[i].point);
                    let originVector = targetss.clone();
                    rayDistance.x = (originVector.x - pointVector.x);
                    rayDistance.y = (originVector.y - pointVector.y);
                    rayDistance.z = (originVector.z - pointVector.z);

                    let rayMag = rayDistance.length();

                    let springLength = THREE.Math.clamp(rayMag - wheelRadius, 0, suspensionMaxLength);
                    //console.log(springLength);
                    let stiffnessForce = stiffness * (suspensionMaxLength - springLength);
                    let damperForce = damper * ((suspensionLengthArray[i] - springLength) / (timeDiff/1000));

                    let suspensionForce2 = new THREE.Vector3();
                    suspensionForce2.x = up.x * (stiffnessForce + damperForce);
                    suspensionForce2.y = up.y * (stiffnessForce + damperForce);
                    suspensionForce2.z = up.z * (stiffnessForce + damperForce);

                    

                    suspensionLengthArray[i] = springLength;

                    let xForce = new THREE.Vector3();
                    

                    let zForce = new THREE.Vector3();
                    zForce.x = forward.x * (moveSpeed*0.5);
                    zForce.y = forward.y * (moveSpeed*0.5);
                    zForce.z = forward.z * (moveSpeed*0.5);

                    /*if(suspensionForce2.length() > 150) {
                        if(crashSound && !crashSound.isPlaying) {
                            crashSound.play();
                        }
                    }*/

                    let localVelocity = new THREE.Vector3();
                    let forwardDir = forward.clone().normalize();
                    let rightDir = right.clone().normalize();
                    physics.getLinearVelocity(vehicle, localVelocity);
                    actualSpeed = localVelocity.dot(forwardDir);
                    let rightDot = localVelocity.dot(rightDir);
                    let forwardVelocity = new THREE.Vector3();
                    let sidewayVelocity = new THREE.Vector3();

                    forwardVelocity.x = forwardDir.x * actualSpeed;
                    forwardVelocity.y = forwardDir.y * actualSpeed;
                    forwardVelocity.z = forwardDir.z * actualSpeed;

                    sidewayVelocity.x = rightDir.x * rightDot;
                    sidewayVelocity.y = rightDir.y * rightDot;
                    sidewayVelocity.z = rightDir.z * rightDot;

                    xForce.x = (right.x * -sidewayVelocity.x);
                    xForce.y = (right.y * -sidewayVelocity.x);
                    xForce.z = (right.z * -sidewayVelocity.x);

                    if(pointArray[i] === pointArray[0]) {

                        if(pointArray[i].distance < (50000000000)) {
                            

                            /*if(keyShift && actualSpeed > 0) {
                                let brakeVec = new THREE.Vector3();
                                brakeVec.x = -forward.x * brakeForce;
                                brakeVec.y = -forward.y * brakeForce;
                                brakeVec.z = -forward.z * brakeForce;
                                physics.addForceAtPos(vehicle, brakeVec, targetss);

                            }*/
                            //physics.addForceAtPos(vehicle, suspensionForce2.multiplyScalar(1), targetss);
                            //console.log(suspensionForce2, "suspension force1");
                            if(pointArray[i] === pointArray[0]) {
                                if(keyA) {
                                    //newRotVec.x = fx * right.x;
                                    //newRotVec.y = fx * right.y;
                                    //newRotVec.z = fx * right.z;
                                }
                                if(keyD) {
                                    //fx -= 5;
                                    //newRotVec.x = fx * right.x;
                                    //newRotVec.y = fx * right.y;
                                    //newRotVec.z = fx * right.z;
                                    
                                }
                                //physics.addForceAtPos(vehicle, new THREE.Vector3(newRotVec.x, newRotVec.y, newRotVec.z), targetss);
                            }

                            if(pointArray[i] === pointArray[1]) {
                                if(keyA) {
                                    //fx += 5;
                                    //newRotVec.x = fx * right.x;
                                    //newRotVec.y = fx * right.y;
                                    //newRotVec.z = fx * right.z;
                                }
                                if(keyD) {
                                    //newRotVec.x = fx * right.x;
                                    //newRotVec.y = fx * right.y;
                                    //newRotVec.z = fx * right.z;
                                    
                                }
                                //physics.addForceAtPos(vehicle, new THREE.Vector3(newRotVec.x, newRotVec.y, newRotVec.z), targetss);
                            }
                            

                            
                        }
                        
                    }
                    if(pointArray[i]) {
                        if(pointArray[i].distance < (springLength + wheelRadius*2)) {
                            let friction = new THREE.Vector3(0,0,-1).applyQuaternion(vehicle.quaternion);

                            if(actualSpeed > 0) {
                              suspensionForce2.add(friction);
                            }

                            //let force = suspensionForce2.multiplyScalar(0.005);
                            physics.addForceAtPos(vehicle, suspensionForce2.multiplyScalar(0.005), targetss);
                            

                            //console.log(suspensionForce2, "suspension force2");
                            // let accVec = new THREE.Vector3();
                            // accVec.x = forward.x * moveSpeed * 0.5;
                            // accVec.y = forward.y * moveSpeed * 0.5;
                            // accVec.z = forward.z * moveSpeed * 0.5;
                            //physics.addForceAtPos(vehicle, accVec, targetss);
                            
                        }
                        
                        //physics.addForceAtPos(vehicle, new THREE.Vector3(-newRotVec.x, -newRotVec.y, -newRotVec.z), targetss);
                        //physics.addForceAtPos(vehicle, new THREE.Vector3(-newPointVec.x, -newPointVec.y, -newPointVec.z), targetss);
                    }

                    let xNewForce = new THREE.Vector3();
                            //localVelocity.applyQuaternion(forward);
                            xNewForce.x = -sidewayVelocity.x * wheelFriction;
                            xNewForce.y = -sidewayVelocity.x * wheelFriction;
                            xNewForce.z = -sidewayVelocity.x * wheelFriction;

                    //physics.addForceAtPos(vehicle, xForce, targetss);
                            

                    

                    if(pointArray[i].distance < (100000)) {
                        if(pointArray[i] === pointArray[2]) {

                           
                              
                            
                            
                            let tempAngular = new THREE.Vector3();
                              //physics.getLinearVelocity(vehicle, tempLinear);
                              physics.getAngularVelocity(vehicle, tempAngular);

                              tempAngular.x *= 0.95;
                              tempAngular.y *= 0.95;
                              tempAngular.z *= 0.95;

                              //tempLinear.x *= 0.95;
                              //tempLinear.y *= 0.95;
                              //tempLinear.z *= 0.95;

                              //angularInput.x *= 0.95;
                              //angularInput.y *= 0.95;
                              //angularInput.z *= 0.95;

                              tempAngular.add(angularInput);

                              //angularInput.z += Math.sin(timeDiff/1000);

                              //const ndc = f => (-0.5 + f) * 2;
                              //let index = 0;
                              //const randomValue = () => ndc(shakeNoise.noise1D(baseTime + timeOffset * index++));

                              // const randomValue = shakeNoise.noise1D(angularInput.z);

                              // let turbulence = Math.random() - 0.5;
                              // console.log(randomValue);
                              // //angularInput.x += randomValue/100;
                              // //angularInput.y += randomValue/100;
                              // angularInput.z += (randomValue/100);

                              

                              physics.setAngularVelocity(vehicle, angularInput, true);
                              physics.setVelocity(vehicle, linearVelocity, true);

                              //physics.addForce(vehicle, forward.multiplyScalar(moveSpeed*60));

                        }
                            
                        }

                    if(pointArray[i]) {

                        wheelArray[i].updateMatrix();
                        

                        let localPos = new THREE.Vector3();
                        app.localToWorld(localPos);
                        localPos.y = pointArray[i].point[1] + wheelRadius;
                        vehicle.worldToLocal(localPos);

                        //console.log(pointArray[i].distance);

                    if(pointArray[i].distance < (suspensionLengthArray[i] + (wheelRadius*2))) {

                      // On ground

                      if(keyX && actualSpeed > 0) {
                        _brake(i);
                        //console.log("braking", actualSpeed)
                      }

                      //console.log("we here");
                      sceneWheels[i].position.setFromMatrixPosition( wheelArray[i].matrixWorld );
                      sceneWheels[i].position.y = pointArray[i].point[1] + wheelRadius;

                      let wheelRpm = Math.abs(moveSpeed) > 1 ? Math.abs(moveSpeed) / 10 : 0;
                      
                      sceneWheels[i].quaternion.copy(vehicle.quaternion);

                      if(sceneWheels[i] === sceneWheels[2]) {
                        //sceneWheels[i].rotateX(wheelRpm);
                      }

                      if(sceneWheels[i] === sceneWheels[3]) {
                        //sceneWheels[i].rotateX(wheelRpm);
                      }
              
                      if(sceneWheels[i] === sceneWheels[0]) {
                        //var clampedRot = THREE.Math.clamp(newRot, -35, 35);
                        //sceneWheels[i].rotateY(THREE.Math.degToRad(clampedRot));
                      }

                      if(sceneWheels[i] === sceneWheels[1]) {
                        //var clampedRot = THREE.Math.clamp(newRot, -35, 35);
                        //sceneWheels[i].rotateY(THREE.Math.degToRad(clampedRot));
                      }
                    }
                    else {
                      //console.log("we here instead");
                      sceneWheels[i].position.setFromMatrixPosition( wheelArray[i].matrixWorld );
                      sceneWheels[i].quaternion.copy(vehicle.quaternion);
                      sceneWheels[i].updateMatrixWorld();
                    }
                      app.position.copy(vehicle.position);
                      app.quaternion.copy(vehicle.quaternion);
                      app.updateMatrixWorld();
                      sceneWheels[i].updateMatrixWorld();
                  }
                }

              //Applying velocities
              let rasa = new THREE.Vector3();
              physics.getLinearVelocity(vehicle, rasa);

              //let wheelForward = new THREE.Vector3(0, 0, 1).applyQuaternion(sceneWheels[0].quaternion);
              
              _updateEngine();
              _updateControlSurfaces();

              let wheelRpm = Math.abs(rasa.length()) > 1 ? Math.abs(rasa.length()) / 100 : 0;

              
            }
          }
        }
        if(app && vehicle && sceneWheels.length >= 4) {
          //Applying physics transform to app
          
          
          

          for (var i = 0; i < sceneWheels.length; i++) {
            
          }
        }
      };
      _updateRide();

    });

    useActivate(() => {

      sitSpec = app.getComponent('sit');
      //console.log("activate 1", app, sitSpec);
      if (sitSpec) {
        let rideMesh = null;

        //console.log("activate 2");

        const {instanceId} = app;

        const rideBone = sitSpec.sitBone ? rideMesh.skeleton.bones.find(bone => bone.name === sitSpec.sitBone) : null;
        const sitAction = {
          type: 'sit',
          time: 0,
          animation: 'stand',
          controllingId: instanceId,
          controllingBone: rideBone,
        };
        localPlayer.setControlAction(sitAction);
        app.wear(false);
      }
    
    });

    app.addEventListener('wearupdate', e => {
      if(e.wear) {
        document.addEventListener("keydown", onDocumentKeyDown, false);
        document.addEventListener('keyup', onDocumentKeyUp);
        localPlayer.avatar.app.visible = false;
      } else {
        document.removeEventListener("keydown", onDocumentKeyDown, false);
        document.removeEventListener('keyup', onDocumentKeyUp);
        localPlayer.avatar.app.visible = true;
        _unwear();
      }
    });

    useCleanup(() => {
      for (const physicsId of physicsIds) {
       physics.removeGeometry(physicsId);
      }
      _unwear();
    });

    return app;
}
