using System;

namespace FinsSim.Core.Spatial
{
    [Serializable]
    public class SixDofMatrix
    {
        public SixDofVector row0;
        public SixDofVector row1;
        public SixDofVector row2;
        public SixDofVector row3;
        public SixDofVector row4;
        public SixDofVector row5;

        public static SixDofMatrix Zero => new SixDofMatrix();

        public static SixDofMatrix Diagonal(SixDofVector diagonal)
        {
            var matrix = new SixDofMatrix();
            for (int i = 0; i < 6; i++)
            {
                matrix[i, i] = diagonal[i];
            }

            return matrix;
        }

        public SixDofMatrix Clone()
        {
            return new SixDofMatrix
            {
                row0 = row0,
                row1 = row1,
                row2 = row2,
                row3 = row3,
                row4 = row4,
                row5 = row5,
            };
        }

        public float this[int row, int column]
        {
            get => GetRow(row)[column];
            set
            {
                SixDofVector vector = GetRow(row);
                vector[column] = value;
                SetRow(row, vector);
            }
        }

        public SixDofVector Multiply(SixDofVector vector)
        {
            return new SixDofVector(
                row0.Dot(vector),
                row1.Dot(vector),
                row2.Dot(vector),
                row3.Dot(vector),
                row4.Dot(vector),
                row5.Dot(vector));
        }

        public SixDofMatrix Abs()
        {
            return new SixDofMatrix
            {
                row0 = row0.Abs(),
                row1 = row1.Abs(),
                row2 = row2.Abs(),
                row3 = row3.Abs(),
                row4 = row4.Abs(),
                row5 = row5.Abs(),
            };
        }

        SixDofVector GetRow(int row)
        {
            switch (row)
            {
                case 0: return row0;
                case 1: return row1;
                case 2: return row2;
                case 3: return row3;
                case 4: return row4;
                case 5: return row5;
                default: throw new IndexOutOfRangeException(nameof(row));
            }
        }

        void SetRow(int row, SixDofVector value)
        {
            switch (row)
            {
                case 0: row0 = value; break;
                case 1: row1 = value; break;
                case 2: row2 = value; break;
                case 3: row3 = value; break;
                case 4: row4 = value; break;
                case 5: row5 = value; break;
                default: throw new IndexOutOfRangeException(nameof(row));
            }
        }
    }
}
