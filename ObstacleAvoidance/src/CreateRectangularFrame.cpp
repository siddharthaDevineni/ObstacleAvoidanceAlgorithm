#include<iostream>

int main( int argc, char* argv[] )
{
    int r, c;

    for( r = 0; r < 20; ++r )
    {
        for( c = 0; c < 80; ++c )
        {
            if( r == 0 || r == 19 )
            {
                std::cout << "*";
            }
            else
            {
                if( c == 0 || c == 79 )
                    std::cout << "*";
                else
                    std::cout << " ";
            }
        }
        std::cout << "\n";
    }

    return 0;
}
