package us.ihmc.euclid.referenceFrame.tools;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

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
      if (rotation1.isIdentity())
      {
         boundingBoxBox3D(rotation2, translation2, inverseTransform2, rotation3, translation3, inverseTransform3, size, boundingBoxToPack);

         if (inverseTransform1)
         {
            boundingBoxToPack.getMinPoint().sub(translation1);
            boundingBoxToPack.getMaxPoint().sub(translation1);
         }
         else
         {
            boundingBoxToPack.getMinPoint().add(translation1);
            boundingBoxToPack.getMaxPoint().add(translation1);
         }
      }
      else if (rotation2.isIdentity())
      {
         boundingBoxBox3D(rotation1, translation1, inverseTransform1, rotation3, translation3, inverseTransform3, size, boundingBoxToPack);

         if (!TupleTools.isTupleZero(translation2, EPSILON))
         {
            double x, y, z;

            if (inverseTransform1)
            {
               x = rotation1.getM00() * translation2.getX() + rotation1.getM10() * translation2.getY() + rotation1.getM20() * translation2.getZ();
               y = rotation1.getM01() * translation2.getX() + rotation1.getM11() * translation2.getY() + rotation1.getM21() * translation2.getZ();
               z = rotation1.getM02() * translation2.getX() + rotation1.getM12() * translation2.getY() + rotation1.getM22() * translation2.getZ();
            }
            else
            {
               x = rotation1.getM00() * translation2.getX() + rotation1.getM01() * translation2.getY() + rotation1.getM02() * translation2.getZ();
               y = rotation1.getM10() * translation2.getX() + rotation1.getM11() * translation2.getY() + rotation1.getM12() * translation2.getZ();
               z = rotation1.getM20() * translation2.getX() + rotation1.getM21() * translation2.getY() + rotation1.getM22() * translation2.getZ();
            }

            if (inverseTransform2)
            {
               x = -x;
               y = -y;
               z = -z;
            }
            boundingBoxToPack.getMinPoint().add(x, y, z);
            boundingBoxToPack.getMaxPoint().add(x, y, z);
         }
      }
      else if (rotation3.isIdentity())
      {
         boundingBoxBox3D(rotation1, translation1, inverseTransform1, rotation2, translation2, inverseTransform2, size, boundingBoxToPack);

         if (!TupleTools.isTupleZero(translation3, EPSILON))
         {
            double x1, y1, z1;
            double x2, y2, z2;

            if (inverseTransform2)
            {
               x1 = rotation2.getM00() * translation3.getX() + rotation2.getM10() * translation3.getY() + rotation2.getM20() * translation3.getZ();
               y1 = rotation2.getM01() * translation3.getX() + rotation2.getM11() * translation3.getY() + rotation2.getM21() * translation3.getZ();
               z1 = rotation2.getM02() * translation3.getX() + rotation2.getM12() * translation3.getY() + rotation2.getM22() * translation3.getZ();
            }
            else
            {
               x1 = rotation2.getM00() * translation3.getX() + rotation2.getM01() * translation3.getY() + rotation2.getM02() * translation3.getZ();
               y1 = rotation2.getM10() * translation3.getX() + rotation2.getM11() * translation3.getY() + rotation2.getM12() * translation3.getZ();
               z1 = rotation2.getM20() * translation3.getX() + rotation2.getM21() * translation3.getY() + rotation2.getM22() * translation3.getZ();
            }

            if (inverseTransform1)
            {
               x2 = rotation1.getM00() * x1 + rotation1.getM10() * y1 + rotation1.getM20() * z1;
               y2 = rotation1.getM01() * x1 + rotation1.getM11() * y1 + rotation1.getM21() * z1;
               z2 = rotation1.getM02() * x1 + rotation1.getM12() * y1 + rotation1.getM22() * z1;
            }
            else
            {
               x2 = rotation1.getM00() * x1 + rotation1.getM01() * y1 + rotation1.getM02() * z1;
               y2 = rotation1.getM10() * x1 + rotation1.getM11() * y1 + rotation1.getM12() * z1;
               z2 = rotation1.getM20() * x1 + rotation1.getM21() * y1 + rotation1.getM22() * z1;
            }

            if (inverseTransform3)
            {
               x2 = -x2;
               y2 = -y2;
               z2 = -z2;
            }
            boundingBoxToPack.getMinPoint().add(x2, y2, z2);
            boundingBoxToPack.getMaxPoint().add(x2, y2, z2);
         }
      }
      else
      {
         double a00, a01, a02;
         double a10, a11, a12;
         double a20, a21, a22;
         double ax, ay, az;

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

            ax = -a00 * translation1.getX() - a01 * translation1.getY() - a02 * translation1.getZ();
            ay = -a10 * translation1.getX() - a11 * translation1.getY() - a12 * translation1.getZ();
            az = -a20 * translation1.getX() - a21 * translation1.getY() - a22 * translation1.getZ();
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

            ax = translation1.getX();
            ay = translation1.getY();
            az = translation1.getZ();
         }

         double b00, b01, b02;
         double b10, b11, b12;
         double b20, b21, b22;
         double bx, by, bz;

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

            bx = -b00 * translation2.getX() - b01 * translation2.getY() - b02 * translation2.getZ();
            by = -b10 * translation2.getX() - b11 * translation2.getY() - b12 * translation2.getZ();
            bz = -b20 * translation2.getX() - b21 * translation2.getY() - b22 * translation2.getZ();
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

            bx = translation2.getX();
            by = translation2.getY();
            bz = translation2.getZ();
         }

         double c00, c01, c02;
         double c10, c11, c12;
         double c20, c21, c22;
         double cx, cy, cz;

         c00 = a00 * b00 + a01 * b10 + a02 * b20;
         c01 = a00 * b01 + a01 * b11 + a02 * b21;
         c02 = a00 * b02 + a01 * b12 + a02 * b22;
         c10 = a10 * b00 + a11 * b10 + a12 * b20;
         c11 = a10 * b01 + a11 * b11 + a12 * b21;
         c12 = a10 * b02 + a11 * b12 + a12 * b22;
         c20 = a20 * b00 + a21 * b10 + a22 * b20;
         c21 = a20 * b01 + a21 * b11 + a22 * b21;
         c22 = a20 * b02 + a21 * b12 + a22 * b22;

         cx = ax + a00 * bx + a01 * by + a02 * bz;
         cy = ay + a10 * bx + a11 * by + a12 * bz;
         cz = az + a20 * bx + a21 * by + a22 * bz;
         boundingBoxBox3D(c00, c01, c02, c10, c11, c12, c20, c21, c22, cx, cy, cz, rotation3, translation3, inverseTransform3, size, boundingBoxToPack);
      }
   }

   private static void boundingBoxBox3D(RotationMatrixReadOnly rotation1, Tuple3DReadOnly translation1, boolean inverseTransform1,
                                        RotationMatrixReadOnly rotation2, Tuple3DReadOnly translation2, boolean inverseTransform2, Vector3DReadOnly size,
                                        BoundingBox3DBasics boundingBoxToPack)
   {
      if (rotation1.isIdentity())
      {
         boundingBoxBox3D(rotation2, translation2, inverseTransform2, size, boundingBoxToPack);

         if (inverseTransform1)
         {
            boundingBoxToPack.getMinPoint().sub(translation1);
            boundingBoxToPack.getMaxPoint().sub(translation1);
         }
         else
         {
            boundingBoxToPack.getMinPoint().add(translation1);
            boundingBoxToPack.getMaxPoint().add(translation1);
         }
      }
      else if (rotation2.isIdentity())
      {
         boundingBoxBox3D(rotation1, translation1, inverseTransform1, size, boundingBoxToPack);

         if (!TupleTools.isTupleZero(translation2, EPSILON))
         {
            double x, y, z;

            if (inverseTransform1)
            {
               x = rotation1.getM00() * translation2.getX() + rotation1.getM10() * translation2.getY() + rotation1.getM20() * translation2.getZ();
               y = rotation1.getM01() * translation2.getX() + rotation1.getM11() * translation2.getY() + rotation1.getM21() * translation2.getZ();
               z = rotation1.getM02() * translation2.getX() + rotation1.getM12() * translation2.getY() + rotation1.getM22() * translation2.getZ();
            }
            else
            {
               x = rotation1.getM00() * translation2.getX() + rotation1.getM01() * translation2.getY() + rotation1.getM02() * translation2.getZ();
               y = rotation1.getM10() * translation2.getX() + rotation1.getM11() * translation2.getY() + rotation1.getM12() * translation2.getZ();
               z = rotation1.getM20() * translation2.getX() + rotation1.getM21() * translation2.getY() + rotation1.getM22() * translation2.getZ();
            }

            if (inverseTransform2)
            {
               x = -x;
               y = -y;
               z = -z;
            }
            boundingBoxToPack.getMinPoint().add(x, y, z);
            boundingBoxToPack.getMaxPoint().add(x, y, z);
         }
      }
      else
      {
         double a00, a01, a02;
         double a10, a11, a12;
         double a20, a21, a22;
         double ax, ay, az;

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

            ax = -a00 * translation1.getX() - a01 * translation1.getY() - a02 * translation1.getZ();
            ay = -a10 * translation1.getX() - a11 * translation1.getY() - a12 * translation1.getZ();
            az = -a20 * translation1.getX() - a21 * translation1.getY() - a22 * translation1.getZ();
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

            ax = translation1.getX();
            ay = translation1.getY();
            az = translation1.getZ();
         }

         boundingBoxBox3D(a00, a01, a02, a10, a11, a12, a20, a21, a22, ax, ay, az, rotation2, translation2, inverseTransform2, size, boundingBoxToPack);
      }
   }

   private static void boundingBoxBox3D(double a00, double a01, double a02, double a10, double a11, double a12, double a20, double a21, double a22, double ax,
                                        double ay, double az, RotationMatrixReadOnly rotation, Tuple3DReadOnly translation, boolean inverseTransform,
                                        Vector3DReadOnly size, BoundingBox3DBasics boundingBoxToPack)
   {
      double b00, b01, b02;
      double b10, b11, b12;
      double b20, b21, b22;
      double bx, by, bz;

      if (inverseTransform)
      {
         b00 = rotation.getM00();
         b01 = rotation.getM10();
         b02 = rotation.getM20();
         b10 = rotation.getM01();
         b11 = rotation.getM11();
         b12 = rotation.getM21();
         b20 = rotation.getM02();
         b21 = rotation.getM12();
         b22 = rotation.getM22();

         bx = -b00 * translation.getX() - b01 * translation.getY() - b02 * translation.getZ();
         by = -b10 * translation.getX() - b11 * translation.getY() - b12 * translation.getZ();
         bz = -b20 * translation.getX() - b21 * translation.getY() - b22 * translation.getZ();
      }
      else
      {
         b00 = rotation.getM00();
         b01 = rotation.getM01();
         b02 = rotation.getM02();
         b10 = rotation.getM10();
         b11 = rotation.getM11();
         b12 = rotation.getM12();
         b20 = rotation.getM20();
         b21 = rotation.getM21();
         b22 = rotation.getM22();

         bx = translation.getX();
         by = translation.getY();
         bz = translation.getZ();
      }

      double c00, c01, c02;
      double c10, c11, c12;
      double c20, c21, c22;
      double cx, cy, cz;

      c00 = a00 * b00 + a01 * b10 + a02 * b20;
      c01 = a00 * b01 + a01 * b11 + a02 * b21;
      c02 = a00 * b02 + a01 * b12 + a02 * b22;
      c10 = a10 * b00 + a11 * b10 + a12 * b20;
      c11 = a10 * b01 + a11 * b11 + a12 * b21;
      c12 = a10 * b02 + a11 * b12 + a12 * b22;
      c20 = a20 * b00 + a21 * b10 + a22 * b20;
      c21 = a20 * b01 + a21 * b11 + a22 * b21;
      c22 = a20 * b02 + a21 * b12 + a22 * b22;

      cx = ax + a00 * bx + a01 * by + a02 * bz;
      cy = ay + a10 * bx + a11 * by + a12 * bz;
      cz = az + a20 * bx + a21 * by + a22 * bz;
      boundingBoxBox3D(c00, c01, c02, c10, c11, c12, c20, c21, c22, cx, cy, cz, size, boundingBoxToPack);
   }

   private static void boundingBoxBox3D(RotationMatrixReadOnly rotation, Tuple3DReadOnly translation, boolean inverseTransform, Vector3DReadOnly size,
                                        BoundingBox3DBasics boundingBoxToPack)
   {
      double m00, m01, m02;
      double m10, m11, m12;
      double m20, m21, m22;
      double x, y, z;

      if (rotation.isIdentity())
      {
         double halfSizeX = 0.5 * size.getX();
         double halfSizeY = 0.5 * size.getY();
         double halfSizeZ = 0.5 * size.getZ();
         x = translation.getX();
         y = translation.getY();
         z = translation.getZ();
         if (inverseTransform)
         {
            x = -x;
            y = -y;
            z = -z;
         }
         boundingBoxToPack.set(x - halfSizeX, y - halfSizeY, z - halfSizeZ, x + halfSizeX, y + halfSizeY, z + halfSizeZ);
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

            x = -m00 * translation.getX() - m01 * translation.getY() - m02 * translation.getZ();
            y = -m10 * translation.getX() - m11 * translation.getY() - m12 * translation.getZ();
            z = -m20 * translation.getX() - m21 * translation.getY() - m22 * translation.getZ();
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

            x = translation.getX();
            y = translation.getY();
            z = translation.getZ();
         }

         boundingBoxBox3D(m00, m01, m02, m10, m11, m12, m20, m21, m22, x, y, z, size, boundingBoxToPack);
      }
   }

   private static void boundingBoxBox3D(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22, double x,
                                        double y, double z, Vector3DReadOnly size, BoundingBox3DBasics boundingBoxToPack)
   {
      double halfSizeX = 0.5 * size.getX();
      double halfSizeY = 0.5 * size.getY();
      double halfSizeZ = 0.5 * size.getZ();
      double xRange = Math.abs(m00) * halfSizeX + Math.abs(m01) * halfSizeY + Math.abs(m02) * halfSizeZ;
      double yRange = Math.abs(m10) * halfSizeX + Math.abs(m11) * halfSizeY + Math.abs(m12) * halfSizeZ;
      double zRange = Math.abs(m20) * halfSizeX + Math.abs(m21) * halfSizeY + Math.abs(m22) * halfSizeZ;

      double maxX = x + xRange;
      double maxY = y + yRange;
      double maxZ = z + zRange;
      double minX = x - xRange;
      double minY = y - yRange;
      double minZ = z - zRange;
      boundingBoxToPack.set(minX, minY, minZ, maxX, maxY, maxZ);
   }
}
