import type {ReactNode} from 'react';
import Link from '@docusaurus/Link';

export default function Navigation(): ReactNode {
  return (
    <nav
      className="sticky top-0 z-50 flex items-center justify-between px-6 md:px-12 py-4 bg-white border-b border-gray-200 shadow-sm"
      style={{
        backgroundColor: '#ffffff',
        height: '64px',
        borderBottom: '1px solid #e5e7eb',
        boxShadow: '0 1px 2px rgba(0, 0, 0, 0.05)'
      }}
    >
      {/* Left: Logo + Title + Textbook Label */}
      <div className="flex items-center gap-3">
        <div className="w-9 h-9 flex items-center justify-center text-xl font-bold">
          🤖
        </div>
        <div className="font-semibold text-gray-900 text-base hidden sm:block">
          Physical AI & Humanoid Robotics
        </div>
        <span className="text-gray-500 text-xs ml-2 hidden md:inline font-normal">Textbook</span>
      </div>

      {/* Right: Links & Actions */}
      <div className="flex items-center gap-6">
        <Link
          to="/intro"
          className="text-gray-600 hover:text-gray-900 text-sm font-medium transition-colors hidden sm:block"
        >
          Course Content
        </Link>
        <a
          href="https://github.com/Shahb/hackthon_humanoid_book"
          target="_blank"
          rel="noopener noreferrer"
          className="text-gray-600 hover:text-gray-900 text-sm font-medium flex items-center gap-1 transition-colors"
        >
          GitHub ↗
        </a>
      </div>
    </nav>
  );
}
