import type {ReactNode} from 'react';
import Link from '@docusaurus/Link';

export default function QuickLinks(): ReactNode {
  const links = [
    {to: '/docs/setup/setup-intro', label: 'Setup Guide'},
    {to: '/docs/setup/setup-workstation', label: 'Workstation'},
    {to: '/docs/setup/setup-edge-kit', label: 'Edge Kit'},
    {to: '/docs/glossary', label: 'Glossary'},
  ];

  return (
    <div className="bg-gray-50 border border-gray-200 rounded-lg p-6 h-fit sticky top-20" style={{backgroundColor: '#f9fafb'}}>
      <h3 className="text-lg font-bold text-gray-900 mb-6">Quick Links</h3>
      <nav className="space-y-4">
        {links.map((link, idx) => (
          <Link
            key={idx}
            to={link.to}
            className="block text-green-700 hover:text-green-800 hover:underline text-base font-medium leading-8 transition-colors"
            style={{color: '#059669'}}
          >
            {link.label}
          </Link>
        ))}
      </nav>
    </div>
  );
}
