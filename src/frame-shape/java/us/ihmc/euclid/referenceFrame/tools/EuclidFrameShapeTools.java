package us.ihmc.euclid.referenceFrame.tools;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCapsule3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCylinder3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameEllipsoid3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameSphere3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * This class provides a variety of tools for performing operations with frame shapes.
 *
 * @author Sylvain Bertrand
 */
public class EuclidFrameShapeTools
{
   private static final double EPSILON = 1.0e-12;

   private EuclidFrameShapeTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   private static interface BoundingBoxRotationPartCalculator<T extends Shape3DReadOnly>
   {
      void computeBoundingBoxZeroRotation(T shape, BoundingBox3DBasics boundingBoxToPack);

      void computeBoundingBox(double m00,
                              double m01,
                              double m02,
                              double m10,
                              double m11,
                              double m12,
                              double m20,
                              double m21,
                              double m22,
                              T shape,
                              BoundingBox3DBasics boundingBoxToPack);
   }

   /**
    * Computes the tightest 3D axis-aligned bounding box that contains a given box 3D.
    *
    * @param box3D             the frame shape to evaluate the bounding box of. Not modified.
    * @param boundingBoxFrame  the reference frame in which the bounding box is to be evaluated.
    * @param boundingBoxToPack the bounding box in which the result is stored. Modified.
    */
   public static void boundingBoxBox3D(FrameBox3DReadOnly box3D, ReferenceFrame boundingBoxFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      boundingBoxBox3D(box3D.getReferenceFrame(), box3D, boundingBoxFrame, boundingBoxToPack);
   }

   /**
    * Computes the tightest 3D axis-aligned bounding box that contains a given box 3D.
    *
    * @param box3DFrame        the reference frame in which the shape is expressed.
    * @param box3D             the shape to evaluate the bounding box of. Not modified.
    * @param boundingBoxFrame  the reference frame in which the bounding box is to be evaluated.
    * @param boundingBoxToPack the bounding box in which the result is stored. Modified.
    */
   public static void boundingBoxBox3D(ReferenceFrame box3DFrame, Box3DReadOnly box3D, ReferenceFrame boundingBoxFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      box3DFrame.verifySameRoots(boundingBoxFrame);

      if (box3DFrame == boundingBoxFrame)
      {
         EuclidShapeTools.boundingBoxBox3D(box3D.getPosition(), box3D.getOrientation(), box3D.getSize(), boundingBoxToPack);
         return;
      }

      Point3DReadOnly shapePosition = box3D.getPosition();
      RotationMatrixReadOnly shapeOrientation = box3D.getOrientation();

      if (box3DFrame.isRootFrame())
      {
         RigidBodyTransform transformFromBBX = boundingBoxFrame.getTransformToRoot();
         Vector3DBasics transFromBBX = transformFromBBX.getTranslation();
         RotationMatrixBasics rotFromBBX = transformFromBBX.getRotation();

         boundingBoxRotationPartGeneric(rotFromBBX, true, shapeOrientation, false, box3D, box3DCalculator, boundingBoxToPack);
         addTranslationPartOfTransforms(rotFromBBX, transFromBBX, true, shapeOrientation, shapePosition, false, boundingBoxToPack);
      }
      else
      {
         RigidBodyTransform transformToRoot = box3DFrame.getTransformToRoot();
         Vector3DBasics transToRoot = transformToRoot.getTranslation();
         RotationMatrixBasics rotToRoot = transformToRoot.getRotation();

         if (boundingBoxFrame.isRootFrame())
         {
            boundingBoxRotationPartGeneric(rotToRoot, false, shapeOrientation, false, box3D, box3DCalculator, boundingBoxToPack);
            addTranslationPartOfTransforms(rotToRoot, transToRoot, false, shapeOrientation, shapePosition, false, boundingBoxToPack);
         }
         else
         {
            RigidBodyTransform transformFromBBX = boundingBoxFrame.getTransformToRoot();
            Vector3DBasics transFromBBX = transformFromBBX.getTranslation();
            RotationMatrixBasics rotFromBBX = transformFromBBX.getRotation();

            boundingBoxRotationPartGeneric(rotFromBBX, true, rotToRoot, false, shapeOrientation, false, box3D, box3DCalculator, boundingBoxToPack);
            addTranslationPartOfTransforms(rotFromBBX,
                                           transFromBBX,
                                           true,
                                           rotToRoot,
                                           transToRoot,
                                           false,
                                           shapeOrientation,
                                           shapePosition,
                                           false,
                                           boundingBoxToPack);
         }
      }
   }

   private static final BoundingBoxRotationPartCalculator<Box3DReadOnly> box3DCalculator = new BoundingBoxRotationPartCalculator<Box3DReadOnly>()
   {
      @Override
      public void computeBoundingBoxZeroRotation(Box3DReadOnly shape, BoundingBox3DBasics boundingBoxToPack)
      {
         double halfSizeX = 0.5 * shape.getSizeX();
         double halfSizeY = 0.5 * shape.getSizeY();
         double halfSizeZ = 0.5 * shape.getSizeZ();
         boundingBoxToPack.set(-halfSizeX, -halfSizeY, -halfSizeZ, halfSizeX, halfSizeY, halfSizeZ);
      }

      @Override
      public void computeBoundingBox(double m00,
                                     double m01,
                                     double m02,
                                     double m10,
                                     double m11,
                                     double m12,
                                     double m20,
                                     double m21,
                                     double m22,
                                     Box3DReadOnly shape,
                                     BoundingBox3DBasics boundingBoxToPack)
      {
         double halfSizeX = 0.5 * shape.getSizeX();
         double halfSizeY = 0.5 * shape.getSizeY();
         double halfSizeZ = 0.5 * shape.getSizeZ();
         double xRange = Math.abs(m00) * halfSizeX + Math.abs(m01) * halfSizeY + Math.abs(m02) * halfSizeZ;
         double yRange = Math.abs(m10) * halfSizeX + Math.abs(m11) * halfSizeY + Math.abs(m12) * halfSizeZ;
         double zRange = Math.abs(m20) * halfSizeX + Math.abs(m21) * halfSizeY + Math.abs(m22) * halfSizeZ;
         boundingBoxToPack.set(-xRange, -yRange, -zRange, xRange, yRange, zRange);
      }
   };

   /**
    * Computes the tightest 3D axis-aligned bounding box that contains a given capsule 3D.
    *
    * @param capsule3D         the frame shape to evaluate the bounding box of. Not modified.
    * @param boundingBoxFrame  the reference frame in which the bounding box is to be evaluated.
    * @param boundingBoxToPack the bounding box in which the result is stored. Modified.
    */
   public static void boundingBoxCapsule3D(FrameCapsule3DReadOnly capsule3D, ReferenceFrame boundingBoxFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      boundingBoxCapsule3D(capsule3D.getReferenceFrame(), capsule3D, boundingBoxFrame, boundingBoxToPack);
   }

   /**
    * Computes the tightest 3D axis-aligned bounding box that contains a given capsule 3D.
    *
    * @param capsule3DFrame    the reference frame in which the shape is expressed.
    * @param capsule3D         the shape to evaluate the bounding box of. Not modified.
    * @param boundingBoxFrame  the reference frame in which the bounding box is to be evaluated.
    * @param boundingBoxToPack the bounding box in which the result is stored. Modified.
    */
   public static void boundingBoxCapsule3D(ReferenceFrame capsule3DFrame,
                                           Capsule3DReadOnly capsule3D,
                                           ReferenceFrame boundingBoxFrame,
                                           BoundingBox3DBasics boundingBoxToPack)
   {
      capsule3DFrame.verifySameRoots(boundingBoxFrame);

      if (capsule3DFrame == boundingBoxFrame)
      {
         EuclidShapeTools.boundingBoxCapsule3D(capsule3D.getPosition(), capsule3D.getAxis(), capsule3D.getLength(), capsule3D.getRadius(), boundingBoxToPack);
         return;
      }

      Point3DReadOnly position = capsule3D.getPosition();

      if (capsule3DFrame.isRootFrame())
      {
         RigidBodyTransform transformFromBBX = boundingBoxFrame.getTransformToRoot();
         Vector3DBasics transFromBBX = transformFromBBX.getTranslation();
         RotationMatrixBasics rotFromBBX = transformFromBBX.getRotation();

         boundingBoxRotationPartGeneric(rotFromBBX, true, capsule3D, capsule3DCalculator, boundingBoxToPack);
         addTranslationPartOfTransforms(rotFromBBX, transFromBBX, true, null, position, false, boundingBoxToPack);
      }
      else
      {
         RigidBodyTransform transformToRoot = capsule3DFrame.getTransformToRoot();
         Vector3DBasics transToRoot = transformToRoot.getTranslation();
         RotationMatrixBasics rotToRoot = transformToRoot.getRotation();

         if (boundingBoxFrame.isRootFrame())
         {
            boundingBoxRotationPartGeneric(rotToRoot, false, capsule3D, capsule3DCalculator, boundingBoxToPack);
            addTranslationPartOfTransforms(rotToRoot, transToRoot, false, null, position, false, boundingBoxToPack);
         }
         else
         {
            RigidBodyTransform transformFromBBX = boundingBoxFrame.getTransformToRoot();
            Vector3DBasics transFromBBX = transformFromBBX.getTranslation();
            RotationMatrixBasics rotFromBBX = transformFromBBX.getRotation();

            boundingBoxRotationPartGeneric(rotFromBBX, true, rotToRoot, false, capsule3D, capsule3DCalculator, boundingBoxToPack);
            addTranslationPartOfTransforms(rotFromBBX, transFromBBX, true, rotToRoot, transToRoot, false, null, position, false, boundingBoxToPack);
         }
      }
   }

   private static final BoundingBoxRotationPartCalculator<Capsule3DReadOnly> capsule3DCalculator = new BoundingBoxRotationPartCalculator<Capsule3DReadOnly>()
   {
      @Override
      public void computeBoundingBoxZeroRotation(Capsule3DReadOnly shape, BoundingBox3DBasics boundingBoxToPack)
      {
         double halfLength = shape.getHalfLength();
         double radius = shape.getRadius();
         Vector3DReadOnly axis = shape.getAxis();
         double axisX = axis.getX();
         double axisY = axis.getY();
         double axisZ = axis.getZ();
         double rangeX = halfLength * Math.abs(axisX) + radius;
         double rangeY = halfLength * Math.abs(axisY) + radius;
         double rangeZ = halfLength * Math.abs(axisZ) + radius;
         boundingBoxToPack.set(-rangeX, -rangeY, -rangeZ, rangeX, rangeY, rangeZ);
      }

      @Override
      public void computeBoundingBox(double m00,
                                     double m01,
                                     double m02,
                                     double m10,
                                     double m11,
                                     double m12,
                                     double m20,
                                     double m21,
                                     double m22,
                                     Capsule3DReadOnly shape,
                                     BoundingBox3DBasics boundingBoxToPack)
      {
         double halfLength = shape.getHalfLength();
         double radius = shape.getRadius();
         Vector3DReadOnly axis = shape.getAxis();
         double axisX = m00 * axis.getX() + m01 * axis.getY() + m02 * axis.getZ();
         double axisY = m10 * axis.getX() + m11 * axis.getY() + m12 * axis.getZ();
         double axisZ = m20 * axis.getX() + m21 * axis.getY() + m22 * axis.getZ();

         double rangeX = halfLength * Math.abs(axisX) + radius;
         double rangeY = halfLength * Math.abs(axisY) + radius;
         double rangeZ = halfLength * Math.abs(axisZ) + radius;
         boundingBoxToPack.set(-rangeX, -rangeY, -rangeZ, rangeX, rangeY, rangeZ);
      }
   };

   /**
    * Computes the tightest 3D axis-aligned bounding box that contains a given cylinder 3D.
    *
    * @param cylinder3D        the frame shape to evaluate the bounding box of. Not modified.
    * @param boundingBoxFrame  the reference frame in which the bounding box is to be evaluated.
    * @param boundingBoxToPack the bounding box in which the result is stored. Modified.
    */
   public static void boundingBoxCylinder3D(FrameCylinder3DReadOnly cylinder3D, ReferenceFrame boundingBoxFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      boundingBoxCylinder3D(cylinder3D.getReferenceFrame(), cylinder3D, boundingBoxFrame, boundingBoxToPack);
   }

   /**
    * Computes the tightest 3D axis-aligned bounding box that contains a given cylinder 3D.
    *
    * @param cylinder3DFrame   the reference frame in which the shape is expressed.
    * @param cylinder3D        the shape to evaluate the bounding box of. Not modified.
    * @param boundingBoxFrame  the reference frame in which the bounding box is to be evaluated.
    * @param boundingBoxToPack the bounding box in which the result is stored. Modified.
    */
   public static void boundingBoxCylinder3D(ReferenceFrame cylinder3DFrame,
                                            Cylinder3DReadOnly cylinder3D,
                                            ReferenceFrame boundingBoxFrame,
                                            BoundingBox3DBasics boundingBoxToPack)
   {
      cylinder3DFrame.verifySameRoots(boundingBoxFrame);

      if (cylinder3DFrame == boundingBoxFrame)
      {
         EuclidShapeTools.boundingBoxCylinder3D(cylinder3D.getPosition(),
                                                cylinder3D.getAxis(),
                                                cylinder3D.getLength(),
                                                cylinder3D.getRadius(),
                                                boundingBoxToPack);
         return;
      }

      Point3DReadOnly position = cylinder3D.getPosition();

      if (cylinder3DFrame.isRootFrame())
      {
         RigidBodyTransform transformFromBBX = boundingBoxFrame.getTransformToRoot();
         Vector3DBasics transFromBBX = transformFromBBX.getTranslation();
         RotationMatrixBasics rotFromBBX = transformFromBBX.getRotation();

         boundingBoxRotationPartGeneric(rotFromBBX, true, cylinder3D, cylinder3DCalculator, boundingBoxToPack);
         addTranslationPartOfTransforms(rotFromBBX, transFromBBX, true, null, position, false, boundingBoxToPack);
      }
      else
      {
         RigidBodyTransform transformToRoot = cylinder3DFrame.getTransformToRoot();
         Vector3DBasics transToRoot = transformToRoot.getTranslation();
         RotationMatrixBasics rotToRoot = transformToRoot.getRotation();

         if (boundingBoxFrame.isRootFrame())
         {
            boundingBoxRotationPartGeneric(rotToRoot, false, cylinder3D, cylinder3DCalculator, boundingBoxToPack);
            addTranslationPartOfTransforms(rotToRoot, transToRoot, false, null, position, false, boundingBoxToPack);
         }
         else
         {
            RigidBodyTransform transformFromBBX = boundingBoxFrame.getTransformToRoot();
            Vector3DBasics transFromBBX = transformFromBBX.getTranslation();
            RotationMatrixBasics rotFromBBX = transformFromBBX.getRotation();

            boundingBoxRotationPartGeneric(rotFromBBX, true, rotToRoot, false, cylinder3D, cylinder3DCalculator, boundingBoxToPack);
            addTranslationPartOfTransforms(rotFromBBX, transFromBBX, true, rotToRoot, transToRoot, false, null, position, false, boundingBoxToPack);
         }
      }
   }

   private static final BoundingBoxRotationPartCalculator<Cylinder3DReadOnly> cylinder3DCalculator = new BoundingBoxRotationPartCalculator<Cylinder3DReadOnly>()
   {
      @Override
      public void computeBoundingBoxZeroRotation(Cylinder3DReadOnly shape, BoundingBox3DBasics boundingBoxToPack)
      {
         double halfLength = shape.getHalfLength();
         double radius = shape.getRadius();
         Vector3DReadOnly axis = shape.getAxis();
         double axisX = axis.getX();
         double axisY = axis.getY();
         double axisZ = axis.getZ();

         double invNormSquared = 1.0 / axis.normSquared();
         double capMinMaxX = Math.max(0.0, radius * EuclidCoreTools.squareRoot(1.0 - axisX * axisX * invNormSquared));
         double capMinMaxY = Math.max(0.0, radius * EuclidCoreTools.squareRoot(1.0 - axisY * axisY * invNormSquared));
         double capMinMaxZ = Math.max(0.0, radius * EuclidCoreTools.squareRoot(1.0 - axisZ * axisZ * invNormSquared));

         double rangeX = halfLength * Math.abs(axisX) + capMinMaxX;
         double rangeY = halfLength * Math.abs(axisY) + capMinMaxY;
         double rangeZ = halfLength * Math.abs(axisZ) + capMinMaxZ;
         boundingBoxToPack.set(-rangeX, -rangeY, -rangeZ, rangeX, rangeY, rangeZ);
      }

      @Override
      public void computeBoundingBox(double m00,
                                     double m01,
                                     double m02,
                                     double m10,
                                     double m11,
                                     double m12,
                                     double m20,
                                     double m21,
                                     double m22,
                                     Cylinder3DReadOnly shape,
                                     BoundingBox3DBasics boundingBoxToPack)
      {
         double halfLength = shape.getHalfLength();
         double radius = shape.getRadius();
         Vector3DReadOnly axis = shape.getAxis();
         double axisX = m00 * axis.getX() + m01 * axis.getY() + m02 * axis.getZ();
         double axisY = m10 * axis.getX() + m11 * axis.getY() + m12 * axis.getZ();
         double axisZ = m20 * axis.getX() + m21 * axis.getY() + m22 * axis.getZ();

         double invNormSquared = 1.0 / axis.normSquared();
         double capMinMaxX = Math.max(0.0, radius * EuclidCoreTools.squareRoot(1.0 - axisX * axisX * invNormSquared));
         double capMinMaxY = Math.max(0.0, radius * EuclidCoreTools.squareRoot(1.0 - axisY * axisY * invNormSquared));
         double capMinMaxZ = Math.max(0.0, radius * EuclidCoreTools.squareRoot(1.0 - axisZ * axisZ * invNormSquared));

         double rangeX = halfLength * Math.abs(axisX) + capMinMaxX;
         double rangeY = halfLength * Math.abs(axisY) + capMinMaxY;
         double rangeZ = halfLength * Math.abs(axisZ) + capMinMaxZ;
         boundingBoxToPack.set(-rangeX, -rangeY, -rangeZ, rangeX, rangeY, rangeZ);
      }
   };

   /**
    * Computes the tightest 3D axis-aligned bounding box that contains a given ellipsoid 3D.
    *
    * @param ellipsoid3D       the frame shape to evaluate the bounding box of. Not modified.
    * @param boundingBoxFrame  the reference frame in which the bounding box is to be evaluated.
    * @param boundingBoxToPack the bounding box in which the result is stored. Modified.
    */
   public static void boundingBoxEllipsoid3D(FrameEllipsoid3DReadOnly ellipsoid3D, ReferenceFrame boundingBoxFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      boundingBoxEllipsoid3D(ellipsoid3D.getReferenceFrame(), ellipsoid3D, boundingBoxFrame, boundingBoxToPack);
   }

   /**
    * Computes the tightest 3D axis-aligned bounding box that contains a given ellipsoid 3D.
    *
    * @param ellipsoid3DFrame  the reference frame in which the shape is expressed.
    * @param ellipsoid3D       the shape to evaluate the bounding box of. Not modified.
    * @param boundingBoxFrame  the reference frame in which the bounding box is to be evaluated.
    * @param boundingBoxToPack the bounding box in which the result is stored. Modified.
    */
   public static void boundingBoxEllipsoid3D(ReferenceFrame ellipsoid3DFrame,
                                             Ellipsoid3DReadOnly ellipsoid3D,
                                             ReferenceFrame boundingBoxFrame,
                                             BoundingBox3DBasics boundingBoxToPack)
   {
      ellipsoid3DFrame.verifySameRoots(boundingBoxFrame);

      if (ellipsoid3DFrame == boundingBoxFrame)
      {
         EuclidShapeTools.boundingBoxEllipsoid3D(ellipsoid3D.getPosition(), ellipsoid3D.getOrientation(), ellipsoid3D.getRadii(), boundingBoxToPack);
         return;
      }

      Point3DReadOnly shapePosition = ellipsoid3D.getPosition();
      RotationMatrixReadOnly shapeOrientation = ellipsoid3D.getOrientation();

      if (ellipsoid3DFrame.isRootFrame())
      {
         RigidBodyTransform transformFromBBX = boundingBoxFrame.getTransformToRoot();
         Vector3DBasics transFromBBX = transformFromBBX.getTranslation();
         RotationMatrixBasics rotFromBBX = transformFromBBX.getRotation();

         boundingBoxRotationPartGeneric(rotFromBBX, true, shapeOrientation, false, ellipsoid3D, ellipsoid3DCalculator, boundingBoxToPack);
         addTranslationPartOfTransforms(rotFromBBX, transFromBBX, true, shapeOrientation, shapePosition, false, boundingBoxToPack);
      }
      else
      {
         RigidBodyTransform transformToRoot = ellipsoid3DFrame.getTransformToRoot();
         Vector3DBasics transToRoot = transformToRoot.getTranslation();
         RotationMatrixBasics rotToRoot = transformToRoot.getRotation();

         if (boundingBoxFrame.isRootFrame())
         {
            boundingBoxRotationPartGeneric(rotToRoot, false, shapeOrientation, false, ellipsoid3D, ellipsoid3DCalculator, boundingBoxToPack);
            addTranslationPartOfTransforms(rotToRoot, transToRoot, false, shapeOrientation, shapePosition, false, boundingBoxToPack);
         }
         else
         {
            RigidBodyTransform transformFromBBX = boundingBoxFrame.getTransformToRoot();
            Vector3DBasics transFromBBX = transformFromBBX.getTranslation();
            RotationMatrixBasics rotFromBBX = transformFromBBX.getRotation();

            boundingBoxRotationPartGeneric(rotFromBBX, true, rotToRoot, false, shapeOrientation, false, ellipsoid3D, ellipsoid3DCalculator, boundingBoxToPack);
            addTranslationPartOfTransforms(rotFromBBX,
                                           transFromBBX,
                                           true,
                                           rotToRoot,
                                           transToRoot,
                                           false,
                                           shapeOrientation,
                                           shapePosition,
                                           false,
                                           boundingBoxToPack);
         }
      }
   }

   private static final BoundingBoxRotationPartCalculator<Ellipsoid3DReadOnly> ellipsoid3DCalculator = new BoundingBoxRotationPartCalculator<Ellipsoid3DReadOnly>()
   {
      @Override
      public void computeBoundingBoxZeroRotation(Ellipsoid3DReadOnly shape, BoundingBox3DBasics boundingBoxToPack)
      {
         Vector3DReadOnly radii = shape.getRadii();
         boundingBoxToPack.set(-radii.getX(), -radii.getY(), -radii.getZ(), radii.getX(), radii.getY(), radii.getZ());
      }

      @Override
      public void computeBoundingBox(double m00,
                                     double m01,
                                     double m02,
                                     double m10,
                                     double m11,
                                     double m12,
                                     double m20,
                                     double m21,
                                     double m22,
                                     Ellipsoid3DReadOnly shape,
                                     BoundingBox3DBasics boundingBoxToPack)
      {
         Vector3DReadOnly radii = shape.getRadii();
         double rx = radii.getX() * radii.getX();
         double ry = radii.getY() * radii.getY();
         double rz = radii.getZ() * radii.getZ();

         double xRange = EuclidCoreTools.squareRoot(m00 * m00 * rx + m01 * m01 * ry + m02 * m02 * rz);
         double yRange = EuclidCoreTools.squareRoot(m10 * m10 * rx + m11 * m11 * ry + m12 * m12 * rz);
         double zRange = EuclidCoreTools.squareRoot(m20 * m20 * rx + m21 * m21 * ry + m22 * m22 * rz);
         boundingBoxToPack.set(-xRange, -yRange, -zRange, xRange, yRange, zRange);
      }
   };

   /**
    * Computes the tightest 3D axis-aligned bounding box that contains a given ramp 3D.
    *
    * @param ramp3D            the frame shape to evaluate the bounding box of. Not modified.
    * @param boundingBoxFrame  the reference frame in which the bounding box is to be evaluated.
    * @param boundingBoxToPack the bounding box in which the result is stored. Modified.
    */
   public static void boundingBoxRamp3D(FrameRamp3DReadOnly ramp3D, ReferenceFrame boundingBoxFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      boundingBoxRamp3D(ramp3D.getReferenceFrame(), ramp3D, boundingBoxFrame, boundingBoxToPack);
   }

   /**
    * Computes the tightest 3D axis-aligned bounding box that contains a given ramp 3D.
    *
    * @param ramp3DFrame       the reference frame in which the shape is expressed.
    * @param ramp3D            the shape to evaluate the bounding box of. Not modified.
    * @param boundingBoxFrame  the reference frame in which the bounding box is to be evaluated.
    * @param boundingBoxToPack the bounding box in which the result is stored. Modified.
    */
   public static void boundingBoxRamp3D(ReferenceFrame ramp3DFrame,
                                        Ramp3DReadOnly ramp3D,
                                        ReferenceFrame boundingBoxFrame,
                                        BoundingBox3DBasics boundingBoxToPack)
   {
      ramp3DFrame.verifySameRoots(boundingBoxFrame);

      if (ramp3DFrame == boundingBoxFrame)
      {
         ramp3D.getBoundingBox(boundingBoxToPack);
         return;
      }

      Point3DReadOnly shapePosition = ramp3D.getPosition();
      RotationMatrixReadOnly shapeOrientation = ramp3D.getOrientation();

      if (ramp3DFrame.isRootFrame())
      {
         RigidBodyTransform transformFromBBX = boundingBoxFrame.getTransformToRoot();
         Vector3DBasics transFromBBX = transformFromBBX.getTranslation();
         RotationMatrixBasics rotFromBBX = transformFromBBX.getRotation();

         boundingBoxRotationPartGeneric(rotFromBBX, true, shapeOrientation, false, ramp3D, ramp3DCalculator, boundingBoxToPack);
         addTranslationPartOfTransforms(rotFromBBX, transFromBBX, true, shapeOrientation, shapePosition, false, boundingBoxToPack);
      }
      else
      {
         RigidBodyTransform transformToRoot = ramp3DFrame.getTransformToRoot();
         Vector3DBasics transToRoot = transformToRoot.getTranslation();
         RotationMatrixBasics rotToRoot = transformToRoot.getRotation();

         if (boundingBoxFrame.isRootFrame())
         {
            boundingBoxRotationPartGeneric(rotToRoot, false, shapeOrientation, false, ramp3D, ramp3DCalculator, boundingBoxToPack);
            addTranslationPartOfTransforms(rotToRoot, transToRoot, false, shapeOrientation, shapePosition, false, boundingBoxToPack);
         }
         else
         {
            RigidBodyTransform transformFromBBX = boundingBoxFrame.getTransformToRoot();
            Vector3DBasics transFromBBX = transformFromBBX.getTranslation();
            RotationMatrixBasics rotFromBBX = transformFromBBX.getRotation();

            boundingBoxRotationPartGeneric(rotFromBBX, true, rotToRoot, false, shapeOrientation, false, ramp3D, ramp3DCalculator, boundingBoxToPack);
            addTranslationPartOfTransforms(rotFromBBX,
                                           transFromBBX,
                                           true,
                                           rotToRoot,
                                           transToRoot,
                                           false,
                                           shapeOrientation,
                                           shapePosition,
                                           false,
                                           boundingBoxToPack);
         }
      }
   }

   private static final BoundingBoxRotationPartCalculator<Ramp3DReadOnly> ramp3DCalculator = new BoundingBoxRotationPartCalculator<Ramp3DReadOnly>()
   {
      @Override
      public void computeBoundingBoxZeroRotation(Ramp3DReadOnly shape, BoundingBox3DBasics boundingBoxToPack)
      {
         double sizeX = shape.getSizeX();
         double halfSizeY = 0.5 * shape.getSizeY();
         double sizeZ = shape.getSizeZ();
         boundingBoxToPack.set(0.0, -halfSizeY, 0.0, sizeX, halfSizeY, sizeZ);
      }

      @Override
      public void computeBoundingBox(double m00,
                                     double m01,
                                     double m02,
                                     double m10,
                                     double m11,
                                     double m12,
                                     double m20,
                                     double m21,
                                     double m22,
                                     Ramp3DReadOnly shape,
                                     BoundingBox3DBasics boundingBoxToPack)
      {
         double sizeX = shape.getSizeX();
         double halfSizeY = 0.5 * shape.getSizeY();
         double sizeZ = shape.getSizeZ();
         double minX = Double.POSITIVE_INFINITY;
         double minY = Double.POSITIVE_INFINITY;
         double minZ = Double.POSITIVE_INFINITY;
         double maxX = Double.NEGATIVE_INFINITY;
         double maxY = Double.NEGATIVE_INFINITY;
         double maxZ = Double.NEGATIVE_INFINITY;

         for (int i = 0; i < 6; i++)
         {
            double xLocal = (i & 2) == 0 ? sizeX : 0.0;
            double yLocal = (i & 1) == 0 ? halfSizeY : -halfSizeY;
            double zLocal = (i & 4) == 0 ? 0.0 : sizeZ;

            double xRoot = m00 * xLocal + m01 * yLocal + m02 * zLocal;
            double yRoot = m10 * xLocal + m11 * yLocal + m12 * zLocal;
            double zRoot = m20 * xLocal + m21 * yLocal + m22 * zLocal;

            minX = Math.min(minX, xRoot);
            minY = Math.min(minY, yRoot);
            minZ = Math.min(minZ, zRoot);
            maxX = Math.max(maxX, xRoot);
            maxY = Math.max(maxY, yRoot);
            maxZ = Math.max(maxZ, zRoot);
         }
         boundingBoxToPack.set(minX, minY, minZ, maxX, maxY, maxZ);
      }
   };

   /**
    * Computes the tightest 3D axis-aligned bounding box that contains a given sphere 3D.
    *
    * @param sphere3D          the frame shape to evaluate the bounding box of. Not modified.
    * @param boundingBoxFrame  the reference frame in which the bounding box is to be evaluated.
    * @param boundingBoxToPack the bounding box in which the result is stored. Modified.
    */
   public static void boundingBoxSphere3D(FrameSphere3DReadOnly sphere3D, ReferenceFrame boundingBoxFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      boundingBoxSphere3D(sphere3D.getReferenceFrame(), sphere3D, boundingBoxFrame, boundingBoxToPack);
   }

   /**
    * Computes the tightest 3D axis-aligned bounding box that contains a given sphere 3D.
    *
    * @param sphere3DFrame     the reference frame in which the shape is expressed.
    * @param sphere3D          the shape to evaluate the bounding box of. Not modified.
    * @param boundingBoxFrame  the reference frame in which the bounding box is to be evaluated.
    * @param boundingBoxToPack the bounding box in which the result is stored. Modified.
    */
   public static void boundingBoxSphere3D(ReferenceFrame sphere3DFrame,
                                          Sphere3DReadOnly sphere3D,
                                          ReferenceFrame boundingBoxFrame,
                                          BoundingBox3DBasics boundingBoxToPack)
   {
      sphere3DFrame.verifySameRoots(boundingBoxFrame);

      if (sphere3DFrame == boundingBoxFrame)
      {
         sphere3D.getBoundingBox(boundingBoxToPack);
         return;
      }

      double minX = -sphere3D.getRadius();
      double minY = -sphere3D.getRadius();
      double minZ = -sphere3D.getRadius();
      double maxX = sphere3D.getRadius();
      double maxY = sphere3D.getRadius();
      double maxZ = sphere3D.getRadius();
      boundingBoxToPack.set(minX, minY, minZ, maxX, maxY, maxZ);

      Point3DReadOnly position = sphere3D.getPosition();

      if (sphere3DFrame.isRootFrame())
      {
         RigidBodyTransform transformFromBBX = boundingBoxFrame.getTransformToRoot();
         Vector3DBasics transFromBBX = transformFromBBX.getTranslation();
         RotationMatrixBasics rotFromBBX = transformFromBBX.getRotation();

         addTranslationPartOfTransforms(rotFromBBX, transFromBBX, true, null, position, false, boundingBoxToPack);
      }
      else
      {
         RigidBodyTransform transformToRoot = sphere3DFrame.getTransformToRoot();
         Vector3DBasics transToRoot = transformToRoot.getTranslation();
         RotationMatrixBasics rotToRoot = transformToRoot.getRotation();

         if (boundingBoxFrame.isRootFrame())
         {
            addTranslationPartOfTransforms(rotToRoot, transToRoot, false, null, position, false, boundingBoxToPack);
         }
         else
         {
            RigidBodyTransform transformFromBBX = boundingBoxFrame.getTransformToRoot();
            Vector3DBasics transFromBBX = transformFromBBX.getTranslation();
            RotationMatrixBasics rotFromBBX = transformFromBBX.getRotation();

            addTranslationPartOfTransforms(rotFromBBX, transFromBBX, true, rotToRoot, transToRoot, false, null, position, false, boundingBoxToPack);
         }
      }
   }

   /**
    * Computes the tightest 3D axis-aligned bounding box that contains a given convex polytope 3D.
    *
    * @param convexPolytope3D  the frame shape to evaluate the bounding box of. Not modified.
    * @param boundingBoxFrame  the reference frame in which the bounding box is to be evaluated.
    * @param boundingBoxToPack the bounding box in which the result is stored. Modified.
    */
   public static void boundingBoxConvexPolytope3D(FrameConvexPolytope3DReadOnly convexPolytope3D,
                                                  ReferenceFrame boundingBoxFrame,
                                                  BoundingBox3DBasics boundingBoxToPack)
   {
      boundingBoxConvexPolytope3D(convexPolytope3D.getReferenceFrame(), convexPolytope3D, boundingBoxFrame, boundingBoxToPack);
   }

   /**
    * Computes the tightest 3D axis-aligned bounding box that contains a given convex polytope 3D.
    *
    * @param convexPolytope3DFrame the reference frame in which the shape is expressed.
    * @param convexPolytope3D      the shape to evaluate the bounding box of. Not modified.
    * @param boundingBoxFrame      the reference frame in which the bounding box is to be evaluated.
    * @param boundingBoxToPack     the bounding box in which the result is stored. Modified.
    */
   public static void boundingBoxConvexPolytope3D(ReferenceFrame convexPolytope3DFrame,
                                                  ConvexPolytope3DReadOnly convexPolytope3D,
                                                  ReferenceFrame boundingBoxFrame,
                                                  BoundingBox3DBasics boundingBoxToPack)
   {
      convexPolytope3DFrame.verifySameRoots(boundingBoxFrame);

      if (convexPolytope3DFrame == boundingBoxFrame)
      {
         boundingBoxToPack.set(convexPolytope3D.getBoundingBox());
         return;
      }

      if (convexPolytope3DFrame.isRootFrame())
      {
         RigidBodyTransform transformFromBBX = boundingBoxFrame.getTransformToRoot();
         Vector3DBasics transFromBBX = transformFromBBX.getTranslation();
         RotationMatrixBasics rotFromBBX = transformFromBBX.getRotation();

         boundingBoxRotationPartGeneric(rotFromBBX, true, convexPolytope3D, convexPolytope3DCalculator, boundingBoxToPack);
         addTranslationPartOfTransform(rotFromBBX, transFromBBX, true, boundingBoxToPack);
      }
      else
      {
         RigidBodyTransform transformToRoot = convexPolytope3DFrame.getTransformToRoot();
         Vector3DBasics transToRoot = transformToRoot.getTranslation();
         RotationMatrixBasics rotToRoot = transformToRoot.getRotation();

         if (boundingBoxFrame.isRootFrame())
         {
            boundingBoxRotationPartGeneric(rotToRoot, false, convexPolytope3D, convexPolytope3DCalculator, boundingBoxToPack);
            addTranslationPartOfTransform(rotToRoot, transToRoot, false, boundingBoxToPack);
         }
         else
         {
            RigidBodyTransform transformFromBBX = boundingBoxFrame.getTransformToRoot();
            Vector3DBasics transFromBBX = transformFromBBX.getTranslation();
            RotationMatrixBasics rotFromBBX = transformFromBBX.getRotation();

            boundingBoxRotationPartGeneric(rotFromBBX, true, rotToRoot, false, convexPolytope3D, convexPolytope3DCalculator, boundingBoxToPack);
            addTranslationPartOfTransforms(rotFromBBX, transFromBBX, true, rotToRoot, transToRoot, false, boundingBoxToPack);
         }
      }
   }

   private static final BoundingBoxRotationPartCalculator<ConvexPolytope3DReadOnly> convexPolytope3DCalculator = new BoundingBoxRotationPartCalculator<ConvexPolytope3DReadOnly>()
   {
      @Override
      public void computeBoundingBoxZeroRotation(ConvexPolytope3DReadOnly shape, BoundingBox3DBasics boundingBoxToPack)
      {
         boundingBoxToPack.set(shape.getBoundingBox());
      }

      private final Vector3D supportDirection = new Vector3D();

      @Override
      public void computeBoundingBox(double m00,
                                     double m01,
                                     double m02,
                                     double m10,
                                     double m11,
                                     double m12,
                                     double m20,
                                     double m21,
                                     double m22,
                                     ConvexPolytope3DReadOnly shape,
                                     BoundingBox3DBasics boundingBoxToPack)
      {
         supportDirection.set(m00, m01, m02);
         Vertex3DReadOnly vertexMaxX = shape.getSupportingVertex(supportDirection);
         double maxX = vertexMaxX.dot(supportDirection);

         supportDirection.set(m10, m11, m12);
         Vertex3DReadOnly vertexMaxY = shape.getSupportingVertex(vertexMaxX, supportDirection);
         double maxY = vertexMaxY.dot(supportDirection);

         supportDirection.set(-m00, -m01, -m02);
         Vertex3DReadOnly vertexMinX = shape.getSupportingVertex(vertexMaxY, supportDirection);
         double minX = -vertexMinX.dot(supportDirection);

         supportDirection.set(-m10, -m11, -m12);
         Vertex3DReadOnly vertexMinY = shape.getSupportingVertex(vertexMinX, supportDirection);
         double minY = -vertexMinY.dot(supportDirection);

         supportDirection.set(m20, m21, m22);
         Vertex3DReadOnly vertexMaxZ = shape.getSupportingVertex(vertexMinX, supportDirection);
         double maxZ = vertexMaxZ.dot(supportDirection);

         supportDirection.set(-m20, -m21, -m22);
         Vertex3DReadOnly vertexMinZ = shape.getSupportingVertex(vertexMinX, supportDirection);
         double minZ = -vertexMinZ.dot(supportDirection);

         boundingBoxToPack.set(minX, minY, minZ, maxX, maxY, maxZ);
      }
   };

   private static void addTranslationPartOfTransforms(RotationMatrixReadOnly rotationPart1,
                                                      Tuple3DReadOnly translationPart1,
                                                      boolean inverseTransform1,
                                                      RotationMatrixReadOnly rotationPart2,
                                                      Tuple3DReadOnly translationPart2,
                                                      boolean inverseTransform2,
                                                      RotationMatrixReadOnly rotationPart3,
                                                      Tuple3DReadOnly translationPart3,
                                                      boolean inverseTransform3,
                                                      BoundingBox3DBasics boundingBoxTransformed)
   {
      if (TupleTools.isTupleZero(translationPart3, EPSILON))
      {
         addTranslationPartOfTransforms(rotationPart1,
                                        translationPart1,
                                        inverseTransform1,
                                        rotationPart2,
                                        translationPart2,
                                        inverseTransform2,
                                        boundingBoxTransformed);
      }
      else
      {
         double minX = boundingBoxTransformed.getMinX();
         double minY = boundingBoxTransformed.getMinY();
         double minZ = boundingBoxTransformed.getMinZ();
         Point3DBasics translationTransformed = boundingBoxTransformed.getMinPoint();
         translationTransformed.setToZero();
         addAndTransform(rotationPart3, translationPart3, inverseTransform3, translationTransformed);
         addAndTransform(rotationPart2, translationPart2, inverseTransform2, translationTransformed);
         addAndTransform(rotationPart1, translationPart1, inverseTransform1, translationTransformed);
         boundingBoxTransformed.getMaxPoint().add(translationTransformed);
         boundingBoxTransformed.getMinPoint().add(minX, minY, minZ);
      }
   }

   private static void addTranslationPartOfTransforms(RotationMatrixReadOnly rotationPart1,
                                                      Tuple3DReadOnly translationPart1,
                                                      boolean inverseTransform1,
                                                      RotationMatrixReadOnly rotationPart2,
                                                      Tuple3DReadOnly translationPart2,
                                                      boolean inverseTransform2,
                                                      BoundingBox3DBasics boundingBoxTransformed)
   {
      if (TupleTools.isTupleZero(translationPart2, EPSILON))
      {
         addTranslationPartOfTransform(rotationPart1, translationPart1, inverseTransform1, boundingBoxTransformed);
      }
      else
      {
         double minX = boundingBoxTransformed.getMinX();
         double minY = boundingBoxTransformed.getMinY();
         double minZ = boundingBoxTransformed.getMinZ();
         boundingBoxTransformed.getMinPoint().setToZero();
         addAndTransform(rotationPart2, translationPart2, inverseTransform2, boundingBoxTransformed.getMinPoint());
         addAndTransform(rotationPart1, translationPart1, inverseTransform1, boundingBoxTransformed.getMinPoint());
         boundingBoxTransformed.getMaxPoint().add(boundingBoxTransformed.getMinPoint());
         boundingBoxTransformed.getMinPoint().add(minX, minY, minZ);
      }
   }

   private static void addTranslationPartOfTransform(RotationMatrixReadOnly rotationPart,
                                                     Tuple3DReadOnly translationPart,
                                                     boolean inverseTransform,
                                                     BoundingBox3DBasics boundingBoxTransformed)
   {
      if (TupleTools.isTupleZero(translationPart, EPSILON))
      {
         return;
      }
      else
      {
         if (rotationPart.isIdentity())
         {
            if (inverseTransform)
            {
               boundingBoxTransformed.getMinPoint().sub(translationPart);
               boundingBoxTransformed.getMaxPoint().sub(translationPart);
            }
            else
            {
               boundingBoxTransformed.getMinPoint().add(translationPart);
               boundingBoxTransformed.getMaxPoint().add(translationPart);
            }
         }
         else
         {
            if (inverseTransform)
            {
               double minX = boundingBoxTransformed.getMinX();
               double minY = boundingBoxTransformed.getMinY();
               double minZ = boundingBoxTransformed.getMinZ();
               boundingBoxTransformed.getMinPoint().setAndNegate(translationPart);
               rotationPart.inverseTransform(boundingBoxTransformed.getMinPoint());
               boundingBoxTransformed.getMaxPoint().add(boundingBoxTransformed.getMinPoint());
               boundingBoxTransformed.getMinPoint().add(minX, minY, minZ);
            }
            else
            {
               boundingBoxTransformed.getMinPoint().add(translationPart);
               boundingBoxTransformed.getMaxPoint().add(translationPart);
            }
         }
      }
   }

   private static void addAndTransform(RotationMatrixReadOnly rotationPart,
                                       Tuple3DReadOnly translationPart,
                                       boolean inverseTransform,
                                       Point3DBasics pointToTransform)
   {
      if (inverseTransform)
      {
         pointToTransform.sub(translationPart);
         if (rotationPart != null)
            rotationPart.inverseTransform(pointToTransform);
      }
      else
      {
         if (rotationPart != null)
            rotationPart.transform(pointToTransform);
         pointToTransform.add(translationPart);
      }
   }

   private static <T extends Shape3DReadOnly> void boundingBoxRotationPartGeneric(RotationMatrixReadOnly rotation1,
                                                                                  boolean inverseTransform1,
                                                                                  RotationMatrixReadOnly rotation2,
                                                                                  boolean inverseTransform2,
                                                                                  RotationMatrixReadOnly rotation3,
                                                                                  boolean inverseTransform3,
                                                                                  T shape,
                                                                                  BoundingBoxRotationPartCalculator<T> calculator,
                                                                                  BoundingBox3DBasics boundingBoxToPack)
   {
      if (rotation1.isIdentity())
      {
         boundingBoxRotationPartGeneric(rotation2, inverseTransform2, rotation3, inverseTransform3, shape, calculator, boundingBoxToPack);
      }
      else if (rotation2.isIdentity())
      {
         boundingBoxRotationPartGeneric(rotation1, inverseTransform1, rotation3, inverseTransform3, shape, calculator, boundingBoxToPack);
      }
      else
      {
         double a00, a01, a02;
         double a10, a11, a12;
         double a20, a21, a22;

         if (inverseTransform1)
         {
            a00 = rotation1.getM00();
            a01 = rotation1.getM10();
            a02 = rotation1.getM20();
            a10 = rotation1.getM01();
            a11 = rotation1.getM11();
            a12 = rotation1.getM21();
            a20 = rotation1.getM02();
            a21 = rotation1.getM12();
            a22 = rotation1.getM22();
         }
         else
         {
            a00 = rotation1.getM00();
            a01 = rotation1.getM01();
            a02 = rotation1.getM02();
            a10 = rotation1.getM10();
            a11 = rotation1.getM11();
            a12 = rotation1.getM12();
            a20 = rotation1.getM20();
            a21 = rotation1.getM21();
            a22 = rotation1.getM22();
         }

         double b00, b01, b02;
         double b10, b11, b12;
         double b20, b21, b22;

         if (inverseTransform2)
         {
            b00 = rotation2.getM00();
            b01 = rotation2.getM10();
            b02 = rotation2.getM20();
            b10 = rotation2.getM01();
            b11 = rotation2.getM11();
            b12 = rotation2.getM21();
            b20 = rotation2.getM02();
            b21 = rotation2.getM12();
            b22 = rotation2.getM22();
         }
         else
         {
            b00 = rotation2.getM00();
            b01 = rotation2.getM01();
            b02 = rotation2.getM02();
            b10 = rotation2.getM10();
            b11 = rotation2.getM11();
            b12 = rotation2.getM12();
            b20 = rotation2.getM20();
            b21 = rotation2.getM21();
            b22 = rotation2.getM22();
         }

         double c00, c01, c02;
         double c10, c11, c12;
         double c20, c21, c22;

         c00 = a00 * b00 + a01 * b10 + a02 * b20;
         c01 = a00 * b01 + a01 * b11 + a02 * b21;
         c02 = a00 * b02 + a01 * b12 + a02 * b22;
         c10 = a10 * b00 + a11 * b10 + a12 * b20;
         c11 = a10 * b01 + a11 * b11 + a12 * b21;
         c12 = a10 * b02 + a11 * b12 + a12 * b22;
         c20 = a20 * b00 + a21 * b10 + a22 * b20;
         c21 = a20 * b01 + a21 * b11 + a22 * b21;
         c22 = a20 * b02 + a21 * b12 + a22 * b22;

         boundingBoxRotationPartGeneric(c00, c01, c02, c10, c11, c12, c20, c21, c22, false, rotation3, inverseTransform3, shape, calculator, boundingBoxToPack);
      }
   }

   private static <T extends Shape3DReadOnly> void boundingBoxRotationPartGeneric(RotationMatrixReadOnly rotation1,
                                                                                  boolean inverseTransform1,
                                                                                  RotationMatrixReadOnly rotation2,
                                                                                  boolean inverseTransform2,
                                                                                  T shape,
                                                                                  BoundingBoxRotationPartCalculator<T> calculator,
                                                                                  BoundingBox3DBasics boundingBoxToPack)
   {
      if (rotation1.isIdentity())
      {
         boundingBoxRotationPartGeneric(rotation2, inverseTransform2, shape, calculator, boundingBoxToPack);
      }
      else
      {
         boundingBoxRotationPartGeneric(rotation1.getM00(),
                                        rotation1.getM01(),
                                        rotation1.getM02(),
                                        rotation1.getM10(),
                                        rotation1.getM11(),
                                        rotation1.getM12(),
                                        rotation1.getM20(),
                                        rotation1.getM21(),
                                        rotation1.getM22(),
                                        inverseTransform1,
                                        rotation2,
                                        inverseTransform2,
                                        shape,
                                        calculator,
                                        boundingBoxToPack);
      }
   }

   private static <T extends Shape3DReadOnly> void boundingBoxRotationPartGeneric(double m00,
                                                                                  double m01,
                                                                                  double m02,
                                                                                  double m10,
                                                                                  double m11,
                                                                                  double m12,
                                                                                  double m20,
                                                                                  double m21,
                                                                                  double m22,
                                                                                  boolean inverseTransform1,
                                                                                  RotationMatrixReadOnly rotation2,
                                                                                  boolean inverseTransform2,
                                                                                  T shape,
                                                                                  BoundingBoxRotationPartCalculator<T> calculator,
                                                                                  BoundingBox3DBasics boundingBoxToPack)
   {
      double a00, a01, a02;
      double a10, a11, a12;
      double a20, a21, a22;

      if (inverseTransform1)
      {
         a00 = m00;
         a01 = m10;
         a02 = m20;
         a10 = m01;
         a11 = m11;
         a12 = m21;
         a20 = m02;
         a21 = m12;
         a22 = m22;
      }
      else
      {
         a00 = m00;
         a01 = m01;
         a02 = m02;
         a10 = m10;
         a11 = m11;
         a12 = m12;
         a20 = m20;
         a21 = m21;
         a22 = m22;
      }

      if (rotation2.isIdentity())
      {
         calculator.computeBoundingBox(a00, a01, a02, a10, a11, a12, a20, a21, a22, shape, boundingBoxToPack);
      }
      else
      {
         double b00, b01, b02;
         double b10, b11, b12;
         double b20, b21, b22;

         if (inverseTransform2)
         {
            b00 = rotation2.getM00();
            b01 = rotation2.getM10();
            b02 = rotation2.getM20();
            b10 = rotation2.getM01();
            b11 = rotation2.getM11();
            b12 = rotation2.getM21();
            b20 = rotation2.getM02();
            b21 = rotation2.getM12();
            b22 = rotation2.getM22();
         }
         else
         {
            b00 = rotation2.getM00();
            b01 = rotation2.getM01();
            b02 = rotation2.getM02();
            b10 = rotation2.getM10();
            b11 = rotation2.getM11();
            b12 = rotation2.getM12();
            b20 = rotation2.getM20();
            b21 = rotation2.getM21();
            b22 = rotation2.getM22();
         }

         double c00, c01, c02;
         double c10, c11, c12;
         double c20, c21, c22;

         c00 = a00 * b00 + a01 * b10 + a02 * b20;
         c01 = a00 * b01 + a01 * b11 + a02 * b21;
         c02 = a00 * b02 + a01 * b12 + a02 * b22;
         c10 = a10 * b00 + a11 * b10 + a12 * b20;
         c11 = a10 * b01 + a11 * b11 + a12 * b21;
         c12 = a10 * b02 + a11 * b12 + a12 * b22;
         c20 = a20 * b00 + a21 * b10 + a22 * b20;
         c21 = a20 * b01 + a21 * b11 + a22 * b21;
         c22 = a20 * b02 + a21 * b12 + a22 * b22;
         calculator.computeBoundingBox(c00, c01, c02, c10, c11, c12, c20, c21, c22, shape, boundingBoxToPack);
      }
   }

   private static <T extends Shape3DReadOnly> void boundingBoxRotationPartGeneric(RotationMatrixReadOnly rotation,
                                                                                  boolean inverseTransform,
                                                                                  T shape,
                                                                                  BoundingBoxRotationPartCalculator<T> calculator,
                                                                                  BoundingBox3DBasics boundingBoxToPack)
   {
      double m00, m01, m02;
      double m10, m11, m12;
      double m20, m21, m22;

      if (rotation.isIdentity())
      {
         calculator.computeBoundingBoxZeroRotation(shape, boundingBoxToPack);
      }
      else
      {
         if (inverseTransform)
         {
            m00 = rotation.getM00();
            m01 = rotation.getM10();
            m02 = rotation.getM20();
            m10 = rotation.getM01();
            m11 = rotation.getM11();
            m12 = rotation.getM21();
            m20 = rotation.getM02();
            m21 = rotation.getM12();
            m22 = rotation.getM22();
         }
         else
         {
            m00 = rotation.getM00();
            m01 = rotation.getM01();
            m02 = rotation.getM02();
            m10 = rotation.getM10();
            m11 = rotation.getM11();
            m12 = rotation.getM12();
            m20 = rotation.getM20();
            m21 = rotation.getM21();
            m22 = rotation.getM22();
         }

         calculator.computeBoundingBox(m00, m01, m02, m10, m11, m12, m20, m21, m22, shape, boundingBoxToPack);
      }
   }
}
