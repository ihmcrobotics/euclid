package us.ihmc.euclid.referenceFrame.tools;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.*;

public class EuclidFrameShapeTools
{
   private static final double EPSILON = 1.0e-12;

   private EuclidFrameShapeTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   public static void boundingBoxBox3D(FrameBox3DReadOnly box3D, ReferenceFrame boundingBoxFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      boundingBoxBox3D(box3D.getReferenceFrame(), box3D, boundingBoxFrame, boundingBoxToPack);
   }

   public static void boundingBoxBox3D(ReferenceFrame box3DFrame, Box3DReadOnly box3D, ReferenceFrame boundingBoxFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      if (box3DFrame == boundingBoxFrame)
      {
         box3D.getBoundingBox(boundingBoxToPack);
         return;
      }

      Vector3DReadOnly size = box3D.getSize();
      Point3DReadOnly shapePosition = box3D.getPosition();
      RotationMatrixReadOnly shapeOrientation = box3D.getOrientation();

      if (box3DFrame.isRootFrame())
      {
         if (boundingBoxFrame.isRootFrame())
         {
            boundingBoxBox3D(shapeOrientation, shapePosition, false, size, boundingBoxToPack);
         }
         else
         {
            RigidBodyTransform transformFromBBX = boundingBoxFrame.getTransformToRoot();
            Vector3DBasics transFromBBX = transformFromBBX.getTranslation();
            RotationMatrixBasics rotFromBBX = transformFromBBX.getRotation();

            boundingBoxBox3D(rotFromBBX, transFromBBX, true, shapeOrientation, shapePosition, false, size, boundingBoxToPack);
         }
      }
      else
      {
         RigidBodyTransform transformToRoot = box3DFrame.getTransformToRoot();
         Vector3DBasics transToRoot = transformToRoot.getTranslation();
         RotationMatrixBasics rotToRoot = transformToRoot.getRotation();

         if (boundingBoxFrame.isRootFrame())
         {
            boundingBoxBox3D(rotToRoot, transToRoot, false, shapeOrientation, shapePosition, false, size, boundingBoxToPack);
         }
         else
         {
            RigidBodyTransform transformFromBBX = boundingBoxFrame.getTransformToRoot();
            Vector3DBasics transFromBBX = transformFromBBX.getTranslation();
            RotationMatrixBasics rotFromBBX = transformFromBBX.getRotation();

            boundingBoxBox3D(rotFromBBX, transFromBBX, true, rotToRoot, transToRoot, false, shapeOrientation, shapePosition, false, size, boundingBoxToPack);
         }
      }
   }

   private static void boundingBoxBox3D(RotationMatrixReadOnly rotation1, Tuple3DReadOnly translation1, boolean inverseTransform1,
                                        RotationMatrixReadOnly rotation2, Tuple3DReadOnly translation2, boolean inverseTransform2,
                                        RotationMatrixReadOnly rotation3, Tuple3DReadOnly translation3, boolean inverseTransform3, Vector3DReadOnly size,
                                        BoundingBox3DBasics boundingBoxToPack)
   {
      boundingBoxBox3D(rotation1, inverseTransform1, rotation2, inverseTransform2, rotation3, inverseTransform3, size, boundingBoxToPack);
      addTranslationPartOfTransforms(rotation1,
                                     translation1,
                                     inverseTransform1,
                                     rotation2,
                                     translation2,
                                     inverseTransform2,
                                     rotation3,
                                     translation3,
                                     inverseTransform3,
                                     boundingBoxToPack);

   }

   private static void boundingBoxBox3D(RotationMatrixReadOnly rotation1, Tuple3DReadOnly translation1, boolean inverseTransform1,
                                        RotationMatrixReadOnly rotation2, Tuple3DReadOnly translation2, boolean inverseTransform2, Vector3DReadOnly size,
                                        BoundingBox3DBasics boundingBoxToPack)
   {
      boundingBoxBox3D(rotation1, inverseTransform1, rotation2, inverseTransform2, size, boundingBoxToPack);
      addTranslationPartOfTransforms(rotation1, translation1, inverseTransform1, rotation2, translation2, inverseTransform2, boundingBoxToPack);
   }

   private static void boundingBoxBox3D(RotationMatrixReadOnly rotation1, Tuple3DReadOnly translation1, boolean inverseTransform1, Vector3DReadOnly size,
                                        BoundingBox3DBasics boundingBoxToPack)
   {
      boundingBoxBox3D(rotation1, inverseTransform1, size, boundingBoxToPack);
      addTranslationPartOfTransform(rotation1, translation1, inverseTransform1, boundingBoxToPack);
   }

   private static void boundingBoxBox3D(RotationMatrixReadOnly rotation1, boolean inverseTransform1, RotationMatrixReadOnly rotation2,
                                        boolean inverseTransform2, RotationMatrixReadOnly rotation3, boolean inverseTransform3, Vector3DReadOnly size,
                                        BoundingBox3DBasics boundingBoxToPack)
   {
      if (rotation1.isIdentity())
      {
         boundingBoxBox3D(rotation2, inverseTransform2, rotation3, inverseTransform3, size, boundingBoxToPack);
      }
      else if (rotation2.isIdentity())
      {
         boundingBoxBox3D(rotation1, inverseTransform1, rotation3, inverseTransform3, size, boundingBoxToPack);
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

         boundingBoxBox3D(c00, c01, c02, c10, c11, c12, c20, c21, c22, false, rotation3, inverseTransform3, size, boundingBoxToPack);
      }
   }

   private static void boundingBoxBox3D(RotationMatrixReadOnly rotation1, boolean inverseTransform1, RotationMatrixReadOnly rotation2,
                                        boolean inverseTransform2, Vector3DReadOnly size, BoundingBox3DBasics boundingBoxToPack)
   {
      if (rotation1.isIdentity())
      {
         boundingBoxBox3D(rotation2, inverseTransform2, size, boundingBoxToPack);
      }
      else
      {
         boundingBoxBox3D(rotation1.getM00(),
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
                          size,
                          boundingBoxToPack);
      }
   }

   private static void boundingBoxBox3D(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22,
                                        boolean inverseTransform1, RotationMatrixReadOnly rotation2, boolean inverseTransform2, Vector3DReadOnly size,
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
         boundingBoxBox3D(a00, a01, a02, a10, a11, a12, a20, a21, a22, size, boundingBoxToPack);
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
         boundingBoxBox3D(c00, c01, c02, c10, c11, c12, c20, c21, c22, size, boundingBoxToPack);
      }
   }

   private static void boundingBoxBox3D(RotationMatrixReadOnly rotation, boolean inverseTransform, Vector3DReadOnly size, BoundingBox3DBasics boundingBoxToPack)
   {
      double m00, m01, m02;
      double m10, m11, m12;
      double m20, m21, m22;

      if (rotation.isIdentity())
      {
         double halfSizeX = 0.5 * size.getX();
         double halfSizeY = 0.5 * size.getY();
         double halfSizeZ = 0.5 * size.getZ();
         boundingBoxToPack.set(-halfSizeX, -halfSizeY, -halfSizeZ, halfSizeX, halfSizeY, halfSizeZ);
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

         boundingBoxBox3D(m00, m01, m02, m10, m11, m12, m20, m21, m22, size, boundingBoxToPack);
      }
   }

   private static void boundingBoxBox3D(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22,
                                        Vector3DReadOnly size, BoundingBox3DBasics boundingBoxToPack)
   {
      double halfSizeX = 0.5 * size.getX();
      double halfSizeY = 0.5 * size.getY();
      double halfSizeZ = 0.5 * size.getZ();
      double xRange = Math.abs(m00) * halfSizeX + Math.abs(m01) * halfSizeY + Math.abs(m02) * halfSizeZ;
      double yRange = Math.abs(m10) * halfSizeX + Math.abs(m11) * halfSizeY + Math.abs(m12) * halfSizeZ;
      double zRange = Math.abs(m20) * halfSizeX + Math.abs(m21) * halfSizeY + Math.abs(m22) * halfSizeZ;
      boundingBoxToPack.set(-xRange, -yRange, -zRange, xRange, yRange, zRange);
   }

   private static void addTranslationPartOfTransforms(RotationMatrixReadOnly rotationPart1, Tuple3DReadOnly translationPart1, boolean inverseTransform1,
                                                      RotationMatrixReadOnly rotationPart2, Tuple3DReadOnly translationPart2, boolean inverseTransform2,
                                                      RotationMatrixReadOnly rotationPart3, Tuple3DReadOnly translationPart3, boolean inverseTransform3,
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

   private static void addTranslationPartOfTransforms(RotationMatrixReadOnly rotationPart1, Tuple3DReadOnly translationPart1, boolean inverseTransform1,
                                                      RotationMatrixReadOnly rotationPart2, Tuple3DReadOnly translationPart2, boolean inverseTransform2,
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

   private static void addTranslationPartOfTransform(RotationMatrixReadOnly rotationPart, Tuple3DReadOnly translationPart, boolean inverseTransform,
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

   private static void addAndTransform(RotationMatrixReadOnly rotationPart, Tuple3DReadOnly translationPart, boolean inverseTransform,
                                       Point3DBasics pointToTransform)
   {
      if (inverseTransform)
      {
         pointToTransform.sub(translationPart);
         rotationPart.inverseTransform(pointToTransform);
      }
      else
      {
         rotationPart.transform(pointToTransform);
         pointToTransform.add(translationPart);
      }
   }
}
