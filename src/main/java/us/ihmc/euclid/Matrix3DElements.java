package us.ihmc.euclid;

import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;

public enum Matrix3DElements
{
   M00
   {
      @Override
      public Axis3D rowAxis()
      {
         return Axis3D.X;
      }

      @Override
      public Axis3D columnAxis()
      {
         return Axis3D.X;
      }

      @Override
      public Matrix3DElements nextRow()
      {
         return M10;
      }

      @Override
      public Matrix3DElements nextColumn()
      {
         return M01;
      }

      @Override
      public Matrix3DElements previousRow()
      {
         return M20;
      }

      @Override
      public Matrix3DElements previousColumn()
      {
         return M02;
      }

      @Override
      public double extract(Matrix3DReadOnly matrix)
      {
         return matrix.getM00();
      }

      @Override
      public void insert(Matrix3DBasics matrix, double value)
      {
         matrix.setM00(value);
      }
   },
   M01
   {
      @Override
      public Axis3D rowAxis()
      {
         return Axis3D.X;
      }

      @Override
      public Axis3D columnAxis()
      {
         return Axis3D.Y;
      }

      @Override
      public Matrix3DElements nextRow()
      {
         return M11;
      }

      @Override
      public Matrix3DElements nextColumn()
      {
         return M02;
      }

      @Override
      public Matrix3DElements previousRow()
      {
         return M21;
      }

      @Override
      public Matrix3DElements previousColumn()
      {
         return M00;
      }

      @Override
      public double extract(Matrix3DReadOnly matrix)
      {
         return matrix.getM01();
      }

      @Override
      public void insert(Matrix3DBasics matrix, double value)
      {
         matrix.setM01(value);
      }
   },
   M02
   {
      @Override
      public Axis3D rowAxis()
      {
         return Axis3D.X;
      }

      @Override
      public Axis3D columnAxis()
      {
         return Axis3D.Z;
      }

      @Override
      public Matrix3DElements nextRow()
      {
         return M12;
      }

      @Override
      public Matrix3DElements nextColumn()
      {
         return M00;
      }

      @Override
      public Matrix3DElements previousRow()
      {
         return M22;
      }

      @Override
      public Matrix3DElements previousColumn()
      {
         return M01;
      }

      @Override
      public double extract(Matrix3DReadOnly matrix)
      {
         return matrix.getM02();
      }

      @Override
      public void insert(Matrix3DBasics matrix, double value)
      {
         matrix.setM02(value);
      }
   },
   M10
   {
      @Override
      public Axis3D rowAxis()
      {
         return Axis3D.Y;
      }

      @Override
      public Axis3D columnAxis()
      {
         return Axis3D.X;
      }

      @Override
      public Matrix3DElements nextRow()
      {
         return M20;
      }

      @Override
      public Matrix3DElements nextColumn()
      {
         return M11;
      }

      @Override
      public Matrix3DElements previousRow()
      {
         return M00;
      }

      @Override
      public Matrix3DElements previousColumn()
      {
         return M12;
      }

      @Override
      public double extract(Matrix3DReadOnly matrix)
      {
         return matrix.getM10();
      }

      @Override
      public void insert(Matrix3DBasics matrix, double value)
      {
         matrix.setM10(value);
      }
   },
   M11
   {
      @Override
      public Axis3D rowAxis()
      {
         return Axis3D.Y;
      }

      @Override
      public Axis3D columnAxis()
      {
         return Axis3D.Y;
      }

      @Override
      public Matrix3DElements nextRow()
      {
         return M21;
      }

      @Override
      public Matrix3DElements nextColumn()
      {
         return M12;
      }

      @Override
      public Matrix3DElements previousRow()
      {
         return M01;
      }

      @Override
      public Matrix3DElements previousColumn()
      {
         return M10;
      }

      @Override
      public double extract(Matrix3DReadOnly matrix)
      {
         return matrix.getM11();
      }

      @Override
      public void insert(Matrix3DBasics matrix, double value)
      {
         matrix.setM11(value);
      }
   },
   M12
   {
      @Override
      public Axis3D rowAxis()
      {
         return Axis3D.Y;
      }

      @Override
      public Axis3D columnAxis()
      {
         return Axis3D.Z;
      }

      @Override
      public Matrix3DElements nextRow()
      {
         return M22;
      }

      @Override
      public Matrix3DElements nextColumn()
      {
         return M10;
      }

      @Override
      public Matrix3DElements previousRow()
      {
         return M02;
      }

      @Override
      public Matrix3DElements previousColumn()
      {
         return M11;
      }

      @Override
      public double extract(Matrix3DReadOnly matrix)
      {
         return matrix.getM12();
      }

      @Override
      public void insert(Matrix3DBasics matrix, double value)
      {
         matrix.setM12(value);
      }
   },
   M20
   {
      @Override
      public Axis3D rowAxis()
      {
         return Axis3D.Z;
      }

      @Override
      public Axis3D columnAxis()
      {
         return Axis3D.X;
      }

      @Override
      public Matrix3DElements nextRow()
      {
         return M00;
      }

      @Override
      public Matrix3DElements nextColumn()
      {
         return M21;
      }

      @Override
      public Matrix3DElements previousRow()
      {
         return M10;
      }

      @Override
      public Matrix3DElements previousColumn()
      {
         return M22;
      }

      @Override
      public double extract(Matrix3DReadOnly matrix)
      {
         return matrix.getM20();
      }

      @Override
      public void insert(Matrix3DBasics matrix, double value)
      {
         matrix.setM20(value);
      }
   },
   M21
   {
      @Override
      public Axis3D rowAxis()
      {
         return Axis3D.Z;
      }

      @Override
      public Axis3D columnAxis()
      {
         return Axis3D.Y;
      }

      @Override
      public Matrix3DElements nextRow()
      {
         return M01;
      }

      @Override
      public Matrix3DElements nextColumn()
      {
         return M22;
      }

      @Override
      public Matrix3DElements previousRow()
      {
         return M11;
      }

      @Override
      public Matrix3DElements previousColumn()
      {
         return M20;
      }

      @Override
      public double extract(Matrix3DReadOnly matrix)
      {
         return matrix.getM21();
      }

      @Override
      public void insert(Matrix3DBasics matrix, double value)
      {
         matrix.setM21(value);
      }
   },
   M22
   {
      @Override
      public Axis3D rowAxis()
      {
         return Axis3D.Z;
      }

      @Override
      public Axis3D columnAxis()
      {
         return Axis3D.Z;
      }

      @Override
      public Matrix3DElements nextRow()
      {
         return M02;
      }

      @Override
      public Matrix3DElements nextColumn()
      {
         return M20;
      }

      @Override
      public Matrix3DElements previousRow()
      {
         return M12;
      }

      @Override
      public Matrix3DElements previousColumn()
      {
         return M21;
      }

      @Override
      public double extract(Matrix3DReadOnly matrix)
      {
         return matrix.getM22();
      }

      @Override
      public void insert(Matrix3DBasics matrix, double value)
      {
         matrix.setM22(value);
      }
   };

   private Matrix3DElements()
   {
   }

   public int row()
   {
      return rowAxis().ordinal();
   }

   public int column()
   {
      return columnAxis().ordinal();
   }

   public abstract Axis3D rowAxis();

   public abstract Axis3D columnAxis();

   public abstract Matrix3DElements nextRow();

   public abstract Matrix3DElements nextColumn();

   public abstract Matrix3DElements previousRow();

   public abstract Matrix3DElements previousColumn();

   public abstract double extract(Matrix3DReadOnly matrix);

   public abstract void insert(Matrix3DBasics matrix, double value);

   public static Matrix3DElements fromAxes3D(Axis3D rowAxis, Axis3D columnAxis)
   {
      return switch (rowAxis)
      {
         case X:
         {
            yield switch (columnAxis)
            {
               case X -> M00;
               case Y -> M01;
               case Z -> M02;
            };
         }
         case Y:
         {
            yield switch (columnAxis)
            {
               case X -> M10;
               case Y -> M11;
               case Z -> M12;
            };
         }
         case Z:
         {
            yield switch (columnAxis)
            {
               case X -> M20;
               case Y -> M21;
               case Z -> M22;
            };
         }
      };
   }
}
