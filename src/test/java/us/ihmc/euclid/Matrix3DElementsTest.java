package us.ihmc.euclid;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;

public class Matrix3DElementsTest
{
   @Test
   public void testExtract()
   {
      Matrix3D matrix = new Matrix3D(1, 2, 3, 4, 5, 6, 7, 8, 9);

      assertEquals(1, Matrix3DElements.M00.extract(matrix));
      assertEquals(2, Matrix3DElements.M01.extract(matrix));
      assertEquals(3, Matrix3DElements.M02.extract(matrix));
      assertEquals(matrix.getM00(), Matrix3DElements.M00.extract(matrix));
      assertEquals(matrix.getM01(), Matrix3DElements.M01.extract(matrix));
      assertEquals(matrix.getM02(), Matrix3DElements.M02.extract(matrix));

      assertEquals(4, Matrix3DElements.M10.extract(matrix));
      assertEquals(5, Matrix3DElements.M11.extract(matrix));
      assertEquals(6, Matrix3DElements.M12.extract(matrix));
      assertEquals(matrix.getM10(), Matrix3DElements.M10.extract(matrix));
      assertEquals(matrix.getM11(), Matrix3DElements.M11.extract(matrix));
      assertEquals(matrix.getM12(), Matrix3DElements.M12.extract(matrix));

      assertEquals(7, Matrix3DElements.M20.extract(matrix));
      assertEquals(8, Matrix3DElements.M21.extract(matrix));
      assertEquals(9, Matrix3DElements.M22.extract(matrix));
      assertEquals(matrix.getM20(), Matrix3DElements.M20.extract(matrix));
      assertEquals(matrix.getM21(), Matrix3DElements.M21.extract(matrix));
      assertEquals(matrix.getM22(), Matrix3DElements.M22.extract(matrix));
   }

   @Test
   public void testInsert()
   {
      Matrix3D matrix = new Matrix3D();

      Matrix3DElements.M00.insert(matrix, 1);
      assertEquals(new Matrix3D(1, 0, 0, 0, 0, 0, 0, 0, 0), matrix);
      Matrix3DElements.M01.insert(matrix, 2);
      assertEquals(new Matrix3D(1, 2, 0, 0, 0, 0, 0, 0, 0), matrix);
      Matrix3DElements.M02.insert(matrix, 3);
      assertEquals(new Matrix3D(1, 2, 3, 0, 0, 0, 0, 0, 0), matrix);

      Matrix3DElements.M10.insert(matrix, 4);
      assertEquals(new Matrix3D(1, 2, 3, 4, 0, 0, 0, 0, 0), matrix);
      Matrix3DElements.M11.insert(matrix, 5);
      assertEquals(new Matrix3D(1, 2, 3, 4, 5, 0, 0, 0, 0), matrix);
      Matrix3DElements.M12.insert(matrix, 6);
      assertEquals(new Matrix3D(1, 2, 3, 4, 5, 6, 0, 0, 0), matrix);

      Matrix3DElements.M20.insert(matrix, 7);
      assertEquals(new Matrix3D(1, 2, 3, 4, 5, 6, 7, 0, 0), matrix);
      Matrix3DElements.M21.insert(matrix, 8);
      assertEquals(new Matrix3D(1, 2, 3, 4, 5, 6, 7, 8, 0), matrix);
      Matrix3DElements.M22.insert(matrix, 9);
      assertEquals(new Matrix3D(1, 2, 3, 4, 5, 6, 7, 8, 9), matrix);
   }

   @Test
   public void testFromAxes3D()
   {
      assertEquals(Matrix3DElements.M00, Matrix3DElements.fromAxes3D(Axis3D.X, Axis3D.X));
      assertEquals(Matrix3DElements.M01, Matrix3DElements.fromAxes3D(Axis3D.X, Axis3D.Y));
      assertEquals(Matrix3DElements.M02, Matrix3DElements.fromAxes3D(Axis3D.X, Axis3D.Z));

      assertEquals(Matrix3DElements.M10, Matrix3DElements.fromAxes3D(Axis3D.Y, Axis3D.X));
      assertEquals(Matrix3DElements.M11, Matrix3DElements.fromAxes3D(Axis3D.Y, Axis3D.Y));
      assertEquals(Matrix3DElements.M12, Matrix3DElements.fromAxes3D(Axis3D.Y, Axis3D.Z));

      assertEquals(Matrix3DElements.M20, Matrix3DElements.fromAxes3D(Axis3D.Z, Axis3D.X));
      assertEquals(Matrix3DElements.M21, Matrix3DElements.fromAxes3D(Axis3D.Z, Axis3D.Y));
      assertEquals(Matrix3DElements.M22, Matrix3DElements.fromAxes3D(Axis3D.Z, Axis3D.Z));
   }

   @Test
   public void testRowAxis()
   {
      assertEquals(Matrix3DElements.M00.rowAxis(), Axis3D.X);
      assertEquals(Matrix3DElements.M01.rowAxis(), Axis3D.X);
      assertEquals(Matrix3DElements.M02.rowAxis(), Axis3D.X);

      assertEquals(Matrix3DElements.M10.rowAxis(), Axis3D.Y);
      assertEquals(Matrix3DElements.M11.rowAxis(), Axis3D.Y);
      assertEquals(Matrix3DElements.M12.rowAxis(), Axis3D.Y);

      assertEquals(Matrix3DElements.M20.rowAxis(), Axis3D.Z);
      assertEquals(Matrix3DElements.M21.rowAxis(), Axis3D.Z);
      assertEquals(Matrix3DElements.M22.rowAxis(), Axis3D.Z);
   }

   @Test
   public void testColumnAxis()
   {
      assertEquals(Matrix3DElements.M00.columnAxis(), Axis3D.X);
      assertEquals(Matrix3DElements.M01.columnAxis(), Axis3D.Y);
      assertEquals(Matrix3DElements.M02.columnAxis(), Axis3D.Z);

      assertEquals(Matrix3DElements.M10.columnAxis(), Axis3D.X);
      assertEquals(Matrix3DElements.M11.columnAxis(), Axis3D.Y);
      assertEquals(Matrix3DElements.M12.columnAxis(), Axis3D.Z);

      assertEquals(Matrix3DElements.M20.columnAxis(), Axis3D.X);
      assertEquals(Matrix3DElements.M21.columnAxis(), Axis3D.Y);
      assertEquals(Matrix3DElements.M22.columnAxis(), Axis3D.Z);
   }

   @Test
   public void testNextRow()
   {
      assertEquals(Matrix3DElements.M00.nextRow(), Matrix3DElements.fromAxes3D(Axis3D.X.next(), Axis3D.X));
      assertEquals(Matrix3DElements.M01.nextRow(), Matrix3DElements.fromAxes3D(Axis3D.X.next(), Axis3D.Y));
      assertEquals(Matrix3DElements.M02.nextRow(), Matrix3DElements.fromAxes3D(Axis3D.X.next(), Axis3D.Z));

      assertEquals(Matrix3DElements.M10.nextRow(), Matrix3DElements.fromAxes3D(Axis3D.Y.next(), Axis3D.X));
      assertEquals(Matrix3DElements.M11.nextRow(), Matrix3DElements.fromAxes3D(Axis3D.Y.next(), Axis3D.Y));
      assertEquals(Matrix3DElements.M12.nextRow(), Matrix3DElements.fromAxes3D(Axis3D.Y.next(), Axis3D.Z));

      assertEquals(Matrix3DElements.M20.nextRow(), Matrix3DElements.fromAxes3D(Axis3D.Z.next(), Axis3D.X));
      assertEquals(Matrix3DElements.M21.nextRow(), Matrix3DElements.fromAxes3D(Axis3D.Z.next(), Axis3D.Y));
      assertEquals(Matrix3DElements.M22.nextRow(), Matrix3DElements.fromAxes3D(Axis3D.Z.next(), Axis3D.Z));
   }

   @Test
   public void testNextColumn()
   {
      assertEquals(Matrix3DElements.M00.nextColumn(), Matrix3DElements.fromAxes3D(Axis3D.X, Axis3D.X.next()));
      assertEquals(Matrix3DElements.M01.nextColumn(), Matrix3DElements.fromAxes3D(Axis3D.X, Axis3D.Y.next()));
      assertEquals(Matrix3DElements.M02.nextColumn(), Matrix3DElements.fromAxes3D(Axis3D.X, Axis3D.Z.next()));
      assertEquals(Matrix3DElements.M10.nextColumn(), Matrix3DElements.fromAxes3D(Axis3D.Y, Axis3D.X.next()));
      assertEquals(Matrix3DElements.M11.nextColumn(), Matrix3DElements.fromAxes3D(Axis3D.Y, Axis3D.Y.next()));
      assertEquals(Matrix3DElements.M12.nextColumn(), Matrix3DElements.fromAxes3D(Axis3D.Y, Axis3D.Z.next()));
      assertEquals(Matrix3DElements.M20.nextColumn(), Matrix3DElements.fromAxes3D(Axis3D.Z, Axis3D.X.next()));
      assertEquals(Matrix3DElements.M21.nextColumn(), Matrix3DElements.fromAxes3D(Axis3D.Z, Axis3D.Y.next()));
      assertEquals(Matrix3DElements.M22.nextColumn(), Matrix3DElements.fromAxes3D(Axis3D.Z, Axis3D.Z.next()));
   }

   @Test
   public void testPreviousRow()
   {
      assertEquals(Matrix3DElements.M00.previousRow(), Matrix3DElements.fromAxes3D(Axis3D.X.previous(), Axis3D.X));
      assertEquals(Matrix3DElements.M01.previousRow(), Matrix3DElements.fromAxes3D(Axis3D.X.previous(), Axis3D.Y));
      assertEquals(Matrix3DElements.M02.previousRow(), Matrix3DElements.fromAxes3D(Axis3D.X.previous(), Axis3D.Z));

      assertEquals(Matrix3DElements.M10.previousRow(), Matrix3DElements.fromAxes3D(Axis3D.Y.previous(), Axis3D.X));
      assertEquals(Matrix3DElements.M11.previousRow(), Matrix3DElements.fromAxes3D(Axis3D.Y.previous(), Axis3D.Y));
      assertEquals(Matrix3DElements.M12.previousRow(), Matrix3DElements.fromAxes3D(Axis3D.Y.previous(), Axis3D.Z));

      assertEquals(Matrix3DElements.M20.previousRow(), Matrix3DElements.fromAxes3D(Axis3D.Z.previous(), Axis3D.X));
      assertEquals(Matrix3DElements.M21.previousRow(), Matrix3DElements.fromAxes3D(Axis3D.Z.previous(), Axis3D.Y));
      assertEquals(Matrix3DElements.M22.previousRow(), Matrix3DElements.fromAxes3D(Axis3D.Z.previous(), Axis3D.Z));
   }

   @Test
   public void testPreviousColumn()
   {
      assertEquals(Matrix3DElements.M00.previousColumn(), Matrix3DElements.fromAxes3D(Axis3D.X, Axis3D.X.previous()));
      assertEquals(Matrix3DElements.M01.previousColumn(), Matrix3DElements.fromAxes3D(Axis3D.X, Axis3D.Y.previous()));
      assertEquals(Matrix3DElements.M02.previousColumn(), Matrix3DElements.fromAxes3D(Axis3D.X, Axis3D.Z.previous()));
      assertEquals(Matrix3DElements.M10.previousColumn(), Matrix3DElements.fromAxes3D(Axis3D.Y, Axis3D.X.previous()));
      assertEquals(Matrix3DElements.M11.previousColumn(), Matrix3DElements.fromAxes3D(Axis3D.Y, Axis3D.Y.previous()));
      assertEquals(Matrix3DElements.M12.previousColumn(), Matrix3DElements.fromAxes3D(Axis3D.Y, Axis3D.Z.previous()));
      assertEquals(Matrix3DElements.M20.previousColumn(), Matrix3DElements.fromAxes3D(Axis3D.Z, Axis3D.X.previous()));
      assertEquals(Matrix3DElements.M21.previousColumn(), Matrix3DElements.fromAxes3D(Axis3D.Z, Axis3D.Y.previous()));
      assertEquals(Matrix3DElements.M22.previousColumn(), Matrix3DElements.fromAxes3D(Axis3D.Z, Axis3D.Z.previous()));
   }
}
